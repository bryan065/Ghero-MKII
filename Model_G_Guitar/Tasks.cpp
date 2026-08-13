#include "Tasks.h"
#include "Globals.h"
#include "Hardware.h"

void IRAM_ATTR adxlTapISR() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(xTapSemaphore, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// -------------------------------------------------------------------
// Core 1 Low Priority Tap Task (Pickguard Slap)
// -------------------------------------------------------------------
void tapTaskCore1(void *pvParameters) {
  for (;;) {
    // Wait for the ISR to signal a tap event
    if (xSemaphoreTake(xTapSemaphore, portMAX_DELAY) == pdTRUE) {

      // Clear interrupt source register on ADXL345 so INT1 drops back HIGH
      if (xSemaphoreTake(xI2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        Wire.beginTransmission(0x53);
        Wire.write(0x30); // INT_SOURCE
        Wire.endTransmission();
        Wire.requestFrom(0x53, 1);

        if (Wire.available()) Wire.read();
        xSemaphoreGive(xI2cMutex);
      }

      if (Serial) Serial.println("[PICKGUARD] Pickguard tap detected!");

      if (compositeHID.isConnected()) {
        // Press HELPER_SLAP (Star Power)
        if (xSemaphoreTake(xBleMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          updateGamepadButton(HELPER_SLAP, true);
          gamepad->sendGamepadReport();
          lastActivityTime = millis();
          xSemaphoreGive(xBleMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Active hold duration for tap button press

        // Release HELPER_SLAP
        if (xSemaphoreTake(xBleMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          updateGamepadButton(HELPER_SLAP, false);
          gamepad->sendGamepadReport();
          xSemaphoreGive(xBleMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(150)); // Debounce window to prevent double taps
      }
    }
  }
}

// -------------------------------------------------------------------
// High Priority ADC Task (Core 1, Priority 4)
// -------------------------------------------------------------------
void adcTaskCore1(void *pvParameters) {
  for (;;) {
    // Continuous sampling of all analog hardware registers
#ifdef WHAMMY_ADC_PIN
    sharedWhammyRaw = analogRead(WHAMMY_ADC_PIN);
#endif

#ifdef PICKUP_ADC_PIN
    sharedPickupRaw = analogRead(PICKUP_ADC_PIN);
#endif

#ifdef BATTERY_ADC_PIN
    sharedBatteryMv = analogReadMilliVolts(BATTERY_ADC_PIN);
#endif

    // Only process if hall sensors are detected
    if (isHallEffectMode) {
      sharedStrumUpRaw = analogRead(STRUM_UP_PIN);
      sharedStrumDownRaw = analogRead(STRUM_DOWN_PIN);
    }

    // Yield 1 ms // 1000Hz polling
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// -------------------------------------------------------------------
// Highest Priority Button & Strum Task (Core 1, Priority 5)
// -------------------------------------------------------------------
void buttonTaskCore1(void *pvParameters) {
  // Variables to manage holding "HELPER_FUNCTION" for force re-calibration
  static uint32_t presetPressStartMs = 0;
  static bool calibrationHandled = false;

  for (;;) {
    // Skip processing if currently running active calibration
    if (isCalibratingActive) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    
    if (compositeHID.isConnected()) {
      uint64_t nowUs = esp_timer_get_time();
      bool stateChanged = false;

      for (size_t i = 0; i < BUTTON_COUNT; i++) {
        // If Hall Effect mode is detected, bypass digital polling on strum pins
        if (isHallEffectMode && (BUTTON_MAP[i].pin == STRUM_UP_PIN || BUTTON_MAP[i].pin == STRUM_DOWN_PIN)) {
          continue;
        }

        // Read active LOW physical state
        bool rawPressed = (digitalRead(BUTTON_MAP[i].pin) == LOW);

        // Standard Button Debounce Routine for ALL buttons
        if (rawPressed != lastDebouncedState[i]) {
          if (nowUs - lastStateChangeUs[i] >= BUTTON_MAP[i].debounceUs) {
            lastDebouncedState[i] = rawPressed;
            lastStateChangeUs[i] = nowUs;

            // Handle press/release for HELPER_FUNCTION tracking
            if (BUTTON_MAP[i].id == HELPER_FUNCTION) {
              if (rawPressed) {
                presetPressStartMs = millis();
                calibrationHandled = false;
              }
              else {
                presetPressStartMs = 0;
              }
            }

            // Update gamepad state on BOTH press AND release
            if (xSemaphoreTake(xBleMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
              updateGamepadButton(BUTTON_MAP[i].id, rawPressed);
              stateChanged = true;
              xSemaphoreGive(xBleMutex);
            }

            // Haptic feedback with mechanical strum keys
            //if (rawPressed && (BUTTON_MAP[i].pin == STRUM_UP_PIN || BUTTON_MAP[i].pin == STRUM_DOWN_PIN)) {
            //  triggerStrumHaptic();
            //}
          }
        }

        // ---
        // Independent 3-Second Hold Check for Manual Re-Calibration
        // ---
        if (BUTTON_MAP[i].id == HELPER_FUNCTION && lastDebouncedState[i] && !calibrationHandled && isHallEffectMode) {
          if (millis() - presetPressStartMs >= 3000) {
            calibrationHandled = true; // Mark handled so it doesn't trigger repeatedly

            if (Serial) Serial.println("[STRUM SYSTEM] Manual 3-Second Hold Detected: Re-Calibrating Strum Sensors!");

            isCalibratingActive = true; 
            calibrateStrumFull(); // Interactive 5-second calibration
            isCalibratingActive = false;
          }
        }
      }
      // -----------------------------------

      // If any button state changed, send the combined report over BLE
      if (stateChanged) {
        // Enforce a minimum interval between BLE reports to prevent queue saturation
        while(millis() - lastBleReportMs < BLE_REPORT_INTERVAL_MS) {
            vTaskDelay(pdMS_TO_TICKS(1)); 
        }

        if (xSemaphoreTake(xBleMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          gamepad->sendGamepadReport();
          lastActivityTime = millis();
          xSemaphoreGive(xBleMutex);
        }
      }
    }

    // Yield 1 ms // 1000Hz polling
    vTaskDelay(pdMS_TO_TICKS(1)); 
  }
}

// -------------------------------------------------------------------
// Hall Effect Strum Task (Core 1, Priority 5)
// -------------------------------------------------------------------
void hallEffectStrumTaskCore1(void *pvParameters) {
  // Too jittery? Increase CONFIRM_SAMPLES to 4 or 5 (adds ~4-5ms latency to initial trigger)
  // Too sluggish? Reduce EMA factor: change (filtered * 3 + raw) >> 2 to (filtered * 2 + raw * 2) >> 2 for less filtering
  // Hysteresis too aggressive? Lower 0.15f to 0.10f
  static int EMA_FACTOR = 3;
  static float HYSTERESIS = 0.15f;

  // Variables for hall effect strum actuation
  static bool hallStrumUpState = false;
  static bool hallStrumDownState = false;

  // EMA filtering for noise reduction (smooth but responsive)
  static int filteredUpRaw = -1;
  static int filteredDownRaw = -1;

  // Consecutive confirmation counters to prevent jitter triggers near threshold
  static int upConfirmCount = 0;
  static int downConfirmCount = 0;
  const int CONFIRM_SAMPLES = 3; // Require N consecutive samples past threshold before triggering

  // Debug tracking variables with EMA smoothing
#ifdef DEBUG
    static float emaUpRaw = 0.0f;
    static float emaDownRaw = 0.0f;
    static float emaUpDelta = 0.0f;
    static float emaDownDelta = 0.0f;
    static bool emaInitialized = false;
    const float EMA_ALPHA = 0.2f;  // Smoothing factor: 0.1 = heavy smoothing, 0.5 = light smoothing
    const int DEBUG_CHANGE_THRESHOLD = 50;  // Increased threshold since values are now smoothed
#endif

  for (;;) {
    // Skip processing if currently running active calibration
    if (isCalibratingActive) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // Prevent values from changing mid execution
    int upRaw = sharedStrumUpRaw;
    int downRaw = sharedStrumDownRaw;

    if (compositeHID.isConnected()) {
      uint64_t nowUs = esp_timer_get_time();
      bool stateChanged = false;

      // DYNAMIC FALLBACK: If raw signal drops near zero, a mechanical switch was pressed
      if (upRaw < 200 || downRaw < 200) {

        if (Serial) Serial.println("[STRUM SYSTEM] Mechanical Switch Press Detected! Falling back to Mechanical Mode...");

        // Seamlessly fall back to mechanical digital polling on the fly
        isHallEffectMode = false;

        // Enable pull ups again
        pinMode(STRUM_UP_PIN, INPUT_PULLUP);
        pinMode(STRUM_DOWN_PIN, INPUT_PULLUP);

        // Then terminate and end the task
        if (hallEffectStrumTaskHandle != NULL) {
          if (Serial) Serial.println("[STRUM SYSTEM] Hall Effect Strum Task Terminated!");

          TaskHandle_t tempHandle = hallEffectStrumTaskHandle;
          hallEffectStrumTaskHandle = NULL; 
          vTaskDelete(tempHandle);
        }
      }
      else {
        // Dynamic scaling relative to saved max travel
        int upThresholdDelta = (int)(strumUpMaxDelta * PRESET_SCALES[0][currentPresetIndex]);
        int downThresholdDelta = (int)(strumDownMaxDelta * PRESET_SCALES[1][currentPresetIndex]);
        
        // PROPORTIONAL HYSTERESIS: Scale with threshold instead of fixed value
        // Gives more noise margin when trigger point is deeper
        int upHysteresis = max(40, (int)(upThresholdDelta * HYSTERESIS));
        int downHysteresis = max(40, (int)(downThresholdDelta * HYSTERESIS));

        // EMA filtering to reduce ADC noise
        if (filteredUpRaw < 0) filteredUpRaw = upRaw;
        if (filteredDownRaw < 0) filteredDownRaw = downRaw;
        filteredUpRaw = (filteredUpRaw * EMA_FACTOR + upRaw) >> 2;
        filteredDownRaw = (filteredDownRaw * EMA_FACTOR + downRaw) >> 2;

        // Calculate absolute deviations relative to resting zero points
        int upDelta = abs(filteredUpRaw - strumUpZeroOffset);
        int downDelta = abs(filteredDownRaw - strumDownZeroOffset);

        // ===== DEBUG OUTPUT =====
#ifdef DEBUG_HALL
        // Initialize EMA on first read to avoid ramp-up from zero
        if (!emaInitialized) {
          emaUpRaw = upRaw;
          emaDownRaw = downRaw;
          emaUpDelta = upDelta;
          emaDownDelta = downDelta;
          emaInitialized = true;
        }

        // Apply Exponential Moving Average
        emaUpRaw = (emaUpRaw * (1.0f - EMA_ALPHA)) + (upRaw * EMA_ALPHA);
        emaDownRaw = (emaDownRaw * (1.0f - EMA_ALPHA)) + (downRaw * EMA_ALPHA);
        emaUpDelta = (emaUpDelta * (1.0f - EMA_ALPHA)) + (upDelta * EMA_ALPHA);
        emaDownDelta = (emaDownDelta * (1.0f - EMA_ALPHA)) + (downDelta * EMA_ALPHA);

        // Only print when smoothed values change noticeably
        if (Serial && (abs(emaUpRaw - upRaw) > DEBUG_CHANGE_THRESHOLD || 
                        abs(emaDownRaw - downRaw) > DEBUG_CHANGE_THRESHOLD ||
                        abs(emaUpDelta - upDelta) > DEBUG_CHANGE_THRESHOLD ||
                        abs(emaDownDelta - downDelta) > DEBUG_CHANGE_THRESHOLD)) {
          Serial.printf("[STRUM DEBUG] UP: raw=%.0f, delta=%.0f, thresh=%d, state=%s | "
                        "DN: raw=%.0f, delta=%.0f, thresh=%d, state=%s\n",
                        emaUpRaw, emaUpDelta, upThresholdDelta, hallStrumUpState ? "PRESSED" : "released",
                        emaDownRaw, emaDownDelta, downThresholdDelta, hallStrumDownState ? "PRESSED" : "released");
        }
#endif
                // ===== END DEBUG OUTPUT =====

        // UP STRUM: Consecutive confirmation for initial trigger, hysteresis for release
        if (upDelta > upThresholdDelta) {
          upConfirmCount++;
        } else if (upDelta > (upThresholdDelta - upHysteresis)) {
          // Within hysteresis band - keep current confirmation or set to 1 if already pressed
          upConfirmCount = max(1, upConfirmCount);
        } else {
          upConfirmCount = 0;
        }

        // DOWN STRUM
        if (downDelta > downThresholdDelta) {
          downConfirmCount++;
        } else if (downDelta > (downThresholdDelta - downHysteresis)) {
          downConfirmCount = max(1, downConfirmCount);
        } else {
          downConfirmCount = 0;
        }

        // State transitions: need CONFIRM_SAMPLES to arm, hysteresis to disarm
        bool newStrumUp = (upConfirmCount >= CONFIRM_SAMPLES) ? true
                      : (hallStrumUpState && (upDelta > (upThresholdDelta - upHysteresis)));
        bool newStrumDown = (downConfirmCount >= CONFIRM_SAMPLES) ? true
                        : (hallStrumDownState && (downDelta > (downThresholdDelta - downHysteresis)));

      if (newStrumUp != hallStrumUpState) {
          hallStrumUpState = newStrumUp;
          if (xSemaphoreTake(xBleMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            updateGamepadButton(HALL_STRUM_UP, hallStrumUpState);
            stateChanged = true;
            xSemaphoreGive(xBleMutex);
          }

          // Trigger tactile kick on strum pluck
          if (hallStrumUpState) triggerStrumHaptic();
        }

        if (newStrumDown != hallStrumDownState) {
          hallStrumDownState = newStrumDown;
          if (xSemaphoreTake(xBleMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            updateGamepadButton(HALL_STRUM_DOWN, hallStrumDownState);
            stateChanged = true;
            xSemaphoreGive(xBleMutex);
          }

          // Trigger tactile kick on strum pluck
          if (hallStrumDownState) triggerStrumHaptic();
        }
      }

      // If any button state changed, send the combined report over BLE
      if (stateChanged) {
        // Enforce a minimum interval between BLE reports to prevent queue saturation
        while(millis() - lastBleReportMs < BLE_REPORT_INTERVAL_MS) {
            vTaskDelay(pdMS_TO_TICKS(1)); 
        }

        if (xSemaphoreTake(xBleMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          gamepad->sendGamepadReport();
          lastActivityTime = millis();
          xSemaphoreGive(xBleMutex);
        }
      }
    }

  // Yield 1ms // 1000Hz polling
  vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// -------------------------------------------------------------------
// NeoPixel Task (Core 0, Priority 1)
// -------------------------------------------------------------------
void rgbTaskCore0(void *pvParameters) {
  uint16_t rainbowHue = 0;
  strip.begin();
  //strip.setBrightness(128);

  for (;;) {
    // --- Calibration Flash State Machine (Highest Priority) ---
    if (calState != CAL_NONE) {
      uint32_t color = strip.Color(0, 0, 0);
      uint32_t elapsed = millis() - calPhaseStartMs;

      switch (calState) {
        case CAL_ZERO:
          color = strip.Color(255, 0, 255); // Solid Magenta
          break;

        case CAL_FULL_STRUM:
          // 1Hz blink: 500ms ON, 500ms OFF
          if ((millis() / 500) % 2 == 0) color = strip.Color(255, 0, 255);
          break;

        case CAL_UP_HOLD:
          // Progressive blink: Interval shrinks from 400ms down to 50ms as elapsed approaches 3000ms
          {
            uint32_t blinkInterval = map(constrain(elapsed, 0, 3000), 0, 3000, 400, 50);
            if ((millis() / blinkInterval) % 5 == 0) color = strip.Color(180, 0, 180);
          }
          break;

        case CAL_DOWN_HOLD:
          // Progressive blink: Interval shrinks from 400ms down to 50ms as elapsed approaches 3000ms
          {
            uint32_t blinkInterval = map(constrain(elapsed, 0, 3000), 0, 3000, 400, 50);
            if ((millis() / blinkInterval) % 5 == 0) color = strip.Color(180, 0, 180);
          }
          break;

        case CAL_SUCCESS_PULSE:
          color = strip.Color(0, 180, 0); // Solid Green for successful section
          break;

        case CAL_DONE_PULSES:
          // 3 rapid green flashes over 1.5 seconds
          if ((millis() / 250) % 2 == 0) color = strip.Color(0, 255, 0);
          break;

        default:
          break;
      }

      for (int i = 0; i < NUM_LEDS; i++) strip.setPixelColor(i, color);
      strip.show();
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    // --- NeoPixel #1: Battery / Charging Status ---
    if (isCharging) {
      strip.setPixelColor(0, strip.Color(0, 0, 80));       // BLUE: Charging
    }
    else if (batteryLevel < 10) {
      // Non-blocking 2Hz blink when below 10%
      bool blinkState = (millis() / 250) % 2;
      if (blinkState) {
        strip.setPixelColor(0, strip.Color(80, 0, 0));    // RED (ON)
      } else {
        strip.setPixelColor(0, strip.Color(0, 0, 0));      // OFF
      }
    } else if (batteryLevel < 30) {
      strip.setPixelColor(0, strip.Color(80, 0, 0));       // RED: Low (<30%)
    } else if (batteryLevel <= 80) {
      strip.setPixelColor(0, strip.Color(0, 80, 0));       // GREEN: Normal (30-80%)
    } else {
      strip.setPixelColor(0, strip.Color(100, 100, 100));  // WHITE: Full (>80%)
    }

    // --- NeoPixels #2 & #3: Strum Bar (Preset Feedback vs. Rainbow Effect) ---
    if (millis() - presetShowStartMs < 1000) {
      // Show the current preset indicator color on both strum LEDs for 1 second
      uint32_t indicatorColor = PRESET_COLORS[currentPresetIndex];
      strip.setPixelColor(1, indicatorColor);
      strip.setPixelColor(2, indicatorColor);
    }
    else {
      // Normal 50Hz Rainbow Cycle
      uint32_t color1 = strip.ColorHSV(rainbowHue);
      uint32_t color2 = strip.ColorHSV(rainbowHue + 8000); // HSV gradient offset

      strip.setPixelColor(1, color1);
      strip.setPixelColor(2, color2);

      rainbowHue += 256; // Rotate rainbow colors
    }

    strip.show();
    vTaskDelay(pdMS_TO_TICKS(20)); // ~50Hz refresh rate
  }
}

// -------------------------------------------------------------------
// ADXL345 Tilt Task (Core 0, Priority 1)
// -------------------------------------------------------------------
void tiltTaskCore0(void *pvParameters) {
  float smoothedPitch = 0.0f;

  for (;;) {
    if (compositeHID.isConnected()) {
      sensors_event_t event;

      // Thread-safe
      if (xSemaphoreTake(xI2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        accel.getEvent(&event);
        xSemaphoreGive(xI2cMutex);
      }

      float ax = event.acceleration.x; // Down neck
      float ay = event.acceleration.y; // Across neck width
      float az = event.acceleration.z; // Perpendicular out guitar face

      // Normalize X vector direction for left-handed playing
      if (isLeftHanded) ax = -ax;

      // Calculate pitch angle of the neck relative to horizontal plane
      // sqrt(ay*ay + az*az) prevents roll on Z from breaking tilt sensitivity
      float perpendicularMag = sqrt(ay * ay + az * az);
      float instantPitch = atan2(ax, perpendicularMag) * 180.0f / M_PI;

      // Initialize the filter on the very first read to prevent a slow ramp-up from 0
      if (smoothedPitch == 0.0f) smoothedPitch = instantPitch;

      // Apply Low-Pass Filter (EMA)
      smoothedPitch = (smoothedPitch * 0.8f) + (instantPitch * 0.2f);

      // Resting posture is typically ~0° to 15°.
      // Active Star Power activation window: 20° to 60° vertical pitch
      int16_t mappedTilt = 0;
      if (smoothedPitch > 20.0f) {
        mappedTilt = (int16_t)map(constrain((int)smoothedPitch, 20, 60), 20, 60, 0, 32767);
      }

      // Send update over Right Stick Y-Axis (setRightThumb(X, Y))
      if (abs(mappedTilt - lastSentTilt) >= HID_DEADBAND) {
        if (xSemaphoreTake(xBleMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
          // Keep X neutral (0), pass mappedTilt to Y-axis
          gamepad->setRightThumb(0, -mappedTilt);  
          gamepad->sendGamepadReport();
          lastSentTilt = mappedTilt;
          xSemaphoreGive(xBleMutex);
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(TILT_INTERVAL_MS)); // 50Hz Polling
  }
}

// -------------------------------------------------------------------
// Whammy Task (Core 0, Priority 1)
//   note: Does not reset sleep timeout
//   note2: This task now also handles the pickup encoder
// -------------------------------------------------------------------
void whammyTaskCore0(void *pvParameters) {
  // Variables for Pickup Settling / Debouncing
  int samples[SAMPLE_COUNT];
  int lastMean = 0;
  int settledPickupRaw = -1;
  uint32_t settleStartTime = 0;
  bool isSettling = false;

  for (;;) {
    if (compositeHID.isConnected()) {
      // Whammy Handling
      int rawWhammy = sharedWhammyRaw;

      if (rawWhammy >= 0) {
        if (rawEma < 0) rawEma = rawWhammy;
        rawEma = (rawEma * 3 + rawWhammy) >> 2;

        int constrainedRaw = constrain(rawEma, RAW_MIN, RAW_MAX);
        int32_t x = map(constrainedRaw, RAW_MIN, RAW_MAX, 0, 32767);

        if (abs(x - lastSentWhammy) >= HID_DEADBAND) {
          if (xSemaphoreTake(xBleMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            gamepad->setLeftThumb(x, 16383);
            gamepad->sendGamepadReport();
            lastSentWhammy = x;
            xSemaphoreGive(xBleMutex);
          }
        }
      }

      // Pickup Encoder handling
      // Fast sampling pass (~10ms total for 10 samples)
      uint32_t total = 0;
      
      for (int i = 0; i < SAMPLE_COUNT; i++) {
        samples[i] = sharedPickupRaw;
        total += samples[i];
        vTaskDelay(pdMS_TO_TICKS(1)); // Yield briefly
      }

      int mean = total / SAMPLE_COUNT;

      // --- Settling / Debounce Logic ---
      // If the value changes by more than 250, the switch is moving
      if (abs(mean - lastMean) > 250) {
        lastMean = mean;
        settleStartTime = millis();
        isSettling = true;
      }

      // If the value has been stable for 150ms, it has settled
      else if (isSettling && (millis() - settleStartTime > 150)) {
        isSettling = false;

        // Only process if the new settled value is distinctly different from the last recorded position
        if (abs(mean - settledPickupRaw) > 150) {
          settledPickupRaw = mean;
          if (Serial) Serial.printf("[PICKUP] Raw Pickup Settled At: %u\n", mean);

          // Map the settled ADC mean to one of the 5 presets
          uint8_t newPresetIndex = 0;
          if (mean < 800)       newPresetIndex = 0; // Pos 1 (~370)
          else if (mean < 1600) newPresetIndex = 1; // Pos 2 (~1240)
          else if (mean < 2500) newPresetIndex = 2; // Pos 3 (~2120)
          else if (mean < 3500) newPresetIndex = 3; // Pos 4 (~3015)
          else                  newPresetIndex = 4; // Pos 5 (~4095)

          // Only update and trigger feedback if the preset actually changed
          if (newPresetIndex != currentPresetIndex) {
            currentPresetIndex = newPresetIndex;

            // Trigger the 1-second RGB indicator
            //presetShowStartMs = millis();

            if (Serial) Serial.printf("[STRUM SYSTEM] Pickup switched to Strum Preset Index: %d\n", currentPresetIndex);

            // Save the new index to Flash
            prefs.begin("ghero", false); // Open namespace in Read/Write mode
            prefs.putUChar("strumPreset", currentPresetIndex);
            prefs.end();
          }
        }
      }
    }

    // Polling rate of ~50Hz (20ms)
    vTaskDelay(pdMS_TO_TICKS(WHAMMY_INTERVAL_MS));
  }
}

// -------------------------------------------------------------------
// Battery Monitoring Task (Core 0, Priority 1)
// -------------------------------------------------------------------
void batteryTaskCore0(void *pvParameters) {
  uint32_t samples[SAMPLE_COUNT];
  uint32_t lastBatteryCheckMs = 0;

  for (;;) {
    // --- Charging Status (Runs every 100ms)
    bool currentChargingState = (digitalRead(CHARGING_PIN) == LOW);
    
    // Immediate LED response if charging status changes
    if (currentChargingState != isCharging) {
      isCharging = currentChargingState;
      if (Serial) Serial.printf("[BATTERY] Charging State Changed: %s\n", isCharging ? "CHARGING" : "NOT CHARGING");
    }

    // --- Voltage / Battery Level (BATTERY_REPORT_INTERVAL)
    if (millis() - lastBatteryCheckMs >= BATTERY_REPORT_INTERVAL || lastBatteryCheckMs == 0) {
      lastBatteryCheckMs = millis();

      uint32_t total = 0;

      // Fast sampling pass (~10ms total for 10 samples)
      for (int i = 0; i < SAMPLE_COUNT; i++) {
        samples[i] = sharedBatteryMv;
        total += samples[i];
        vTaskDelay(pdMS_TO_TICKS(1)); // Yield briefly
      }

      float mean = (float)total / SAMPLE_COUNT;

      // Variance/ripple calculation for presence detection
      float variance = 0;
      for (int i = 0; i < SAMPLE_COUNT; i++) {
        variance += pow(samples[i] - mean, 2);
      }
      float stdDev = sqrt(variance / SAMPLE_COUNT);

      // Pin voltage range for 1/2 divider is ~1600mV (3.2V) to ~2100mV (4.2V)
      bool isBatConnected = !(mean < 1200.0f || mean > 2300.0f || stdDev > 80.0f);

      uint8_t newLevel = 0;
      if (isBatConnected) {
        newLevel = getBatteryChargeLevel((uint32_t)mean, BATTERY_CURVE);
      }
      else {
        if (Serial) Serial.printf("[BATTERY] Battery disconnected or noisy signal! Mean: %.1f mV, StdDev: %.1f\n", mean, stdDev);
      }

      // Update global battery state
      batteryLevel = newLevel;

      // Report battery percentage over BLE if connected
      if (compositeHID.isConnected()) {
        if (xSemaphoreTake(xBleMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
          compositeHID.setBatteryLevel(newLevel);
          gamepad->sendGamepadReport();
          xSemaphoreGive(xBleMutex);
        }
      }

      if (Serial) Serial.printf("[BATTERY] Detected: %d | Level: %d%% | Pin Mean: %.1f mV\n", isBatConnected, batteryLevel, mean);
    }

    // Yield
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}