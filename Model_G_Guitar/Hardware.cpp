#include "Hardware.h"
#include "Globals.h"

// -------------------------------------------------------------------
// Handle button ID to gamepad inputs
// -------------------------------------------------------------------
void updateGamepadButton(uint16_t id, bool isPressed) {
  switch (id) {
    case HELPER_LT:         gamepad->setLeftTrigger(isPressed  ? XBOX_TRIGGER_MAX : XBOX_TRIGGER_MIN); break;
    case HELPER_RT:         gamepad->setRightTrigger(isPressed ? XBOX_TRIGGER_MAX : XBOX_TRIGGER_MIN); break;
    case HELPER_DPAD_UP:    gamepad->pressDPadDirectionFlag(isPressed ? NORTH : NONE); break;
    case HELPER_DPAD_DOWN:  gamepad->pressDPadDirectionFlag(isPressed ? SOUTH : NONE); break;
    case HELPER_DPAD_LEFT:  gamepad->pressDPadDirectionFlag(isPressed ? WEST  : NONE); break;
    case HELPER_DPAD_RIGHT: gamepad->pressDPadDirectionFlag(isPressed ? EAST  : NONE); break;
    case HELPER_FUNCTION:   break;
    default:
      if (isPressed) {  gamepad->press(id);   }
      else           {  gamepad->release(id); }
  }
}

// -------------------------------------------------------------------
// Calibrate hall sensor 0-values for 1 second
// -------------------------------------------------------------------
void calibrateStrumZeroOnly() {
  if (!isHallEffectMode) return;

  if (Serial) Serial.println("[STRUM SYSTEM] Calibrating Zero...");

  uint32_t calibrationStartTime = millis();
  uint64_t sumUp = 0;
  uint64_t sumDown = 0;
  uint32_t samplesCount = 0;

  while (millis() - calibrationStartTime < 150) {
    sumUp += sharedStrumUpRaw;
    sumDown += sharedStrumDownRaw;
    samplesCount++;
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  if (samplesCount > 0) {
    strumUpZeroOffset = sumUp / samplesCount;
    strumDownZeroOffset = sumDown / samplesCount;
  }

  if (Serial) Serial.printf("[STRUM SYSTEM] Zero Position - Up: %d | Down: %d\n", strumUpZeroOffset, strumDownZeroOffset);
}

// -------------------------------------------------------------------
// Calibrate hall sensor (Full calibration)
// -------------------------------------------------------------------
void calibrateStrumFull() {
  if (!isHallEffectMode) return;

  // Double click max power on entry
  playHapticEffect(10); // Effect 10: Double Click 100%

  // Zero-offset resting point
  calState = CAL_ZERO;
  calibrateStrumZeroOnly();
  delay(3000);

  // Maximum travel range capture window (5 seconds)
  if (Serial) Serial.println("[STRUM SYSTEM] Calibrating Maximum range...");
  if (Serial) Serial.println("[STRUM SYSTEM] Move the strum bar up/down for the next 5 seconds...");

  calState = CAL_FULL_STRUM;
  calPhaseStartMs = millis();
  int maxObservedUpDelta = 0;
  int maxObservedDownDelta = 0;

  while (millis() - calPhaseStartMs < 15000) {
    int curUpDelta = abs(sharedStrumUpRaw - strumUpZeroOffset);
    int curDownDelta = abs(sharedStrumDownRaw - strumDownZeroOffset);

    if (curUpDelta > maxObservedUpDelta)     maxObservedUpDelta = curUpDelta;
    if (curDownDelta > maxObservedDownDelta) maxObservedDownDelta = curDownDelta;

    lastActivityTime = millis();
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  strumUpMaxDelta = max(maxObservedUpDelta, 200);
  strumDownMaxDelta = max(maxObservedDownDelta, 200);

  // Double click normal power on section complete
  playHapticEffect(11); // Effect 11: Double Click 60%
  calState = CAL_SUCCESS_PULSE;
  //vTaskDelay(pdMS_TO_TICKS(1000));
  delay(5000);

  // Custom UP Threshold Hold (3 seconds)
  if (Serial) Serial.println("[STRUM SYSTEM] Calibrating custom up threshold...");
  if (Serial) Serial.println("[STRUM SYSTEM] Hold the strum in the up position");

  calState = CAL_UP_HOLD;
  calPhaseStartMs = millis();
  uint64_t upHoldSum = 0;
  uint32_t upHoldSamples = 0;
  uint32_t lastRtpUpdateMs = 0;

  // Switch haptic motor to Real-Time Playback for continuous rumble
  if (hapticInitialized) haptic.setMode(DRV2605_MODE_REALTIME);

  while (millis() - calPhaseStartMs < 3000) {
    uint32_t elapsed = millis() - calPhaseStartMs;

    // Slowly build up vibration intensity every 50ms
    if (elapsed - lastRtpUpdateMs > 50) {
      uint8_t intensity = map(constrain(elapsed, 0, 3000), 0, 3000, 15, 127);
      setHapticRTP(intensity);
      lastRtpUpdateMs = elapsed;
    }

    upHoldSum += sharedStrumUpRaw;
    upHoldSamples++;
    lastActivityTime = millis();
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  // Turn off RTP rumble and return to internal sequence trigger mode
  if (hapticInitialized) {
    setHapticRTP(0);
    haptic.setMode(DRV2605_MODE_INTTRIG);
  }

  // Calculate Up Multiplier
  float customUpMult = 0.50f;
  if (upHoldSamples > 0) {
    int meanUp = upHoldSum / upHoldSamples;
    int holdDelta = abs(meanUp - strumUpZeroOffset);
    customUpMult = constrain((float)holdDelta / (float)strumUpMaxDelta, 0.05f, 1.0f);
  }

  // Double click normal power on section complete
  playHapticEffect(11); 
  calState = CAL_SUCCESS_PULSE;
  //vTaskDelay(pdMS_TO_TICKS(1000));
  delay(3000);

  // Custom DOWN Threshold Hold (3 seconds)
  if (Serial) Serial.println("[STRUM SYSTEM] Calibrating custom down threshold...");
  if (Serial) Serial.println("[STRUM SYSTEM] Hold the strum in the down position");

  calState = CAL_DOWN_HOLD;
  calPhaseStartMs = millis();
  uint64_t downHoldSum = 0;
  uint32_t downHoldSamples = 0;
  lastRtpUpdateMs = 0;

  if (hapticInitialized) haptic.setMode(DRV2605_MODE_REALTIME);

  while (millis() - calPhaseStartMs < 3000) {
    uint32_t elapsed = millis() - calPhaseStartMs;

    if (elapsed - lastRtpUpdateMs > 50) {
      uint8_t intensity = map(constrain(elapsed, 0, 3000), 0, 3000, 15, 127);
      setHapticRTP(intensity);
      lastRtpUpdateMs = elapsed;
    }

    downHoldSum += sharedStrumDownRaw;
    downHoldSamples++;
    lastActivityTime = millis();
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  if (hapticInitialized) {
    setHapticRTP(0);
    haptic.setMode(DRV2605_MODE_INTTRIG);
  }

  // Calculate Down Multiplier
  float customDownMult = 0.50f;
  if (downHoldSamples > 0) {
    int meanDown = downHoldSum / downHoldSamples;
    int holdDelta = abs(meanDown - strumDownZeroOffset);
    customDownMult = constrain((float)holdDelta / (float)strumDownMaxDelta, 0.05f, 1.0f);
  }

  // Inject to Array and Save to Flash
  PRESET_SCALES[0][0] = customUpMult;
  PRESET_SCALES[1][0] = customDownMult;

  prefs.begin("ghero", false);
  prefs.putInt("strumUpMax", strumUpMaxDelta);
  prefs.putInt("strumDownMax", strumDownMaxDelta);
  prefs.putFloat("custUpMult", customUpMult);
  prefs.putFloat("custDwnMult", customDownMult);
  prefs.end();

  // Double click normal power on section complete
  playHapticEffect(11); 
  calState = CAL_SUCCESS_PULSE;
  //vTaskDelay(pdMS_TO_TICKS(1000));
  delay(3000);

  if (Serial) {
    Serial.printf("[STRUM SYSTEM] Calibration Complete!\n");
    Serial.printf("  Up Max: %d | Up Custom Mult: %.2f\n", strumUpMaxDelta, customUpMult);
    Serial.printf("  Down Max: %d | Down Custom Mult: %.2f\n", strumDownMaxDelta, customDownMult);
  }

  // Final Sequence -> Sync 3 strong haptic clicks directly with the 3 green LED flashes
  calState = CAL_DONE_PULSES;
  
  for (int i = 0; i < 3; i++) {
    playHapticEffect(1); // Effect 1: Strong Sharp Click 100%
    vTaskDelay(pdMS_TO_TICKS(500)); // 500ms perfectly aligns with the NeoPixel (millis() / 250) blink math
  }

  calState = CAL_NONE; 
}

// -------------------------------------------------------------------
// Detects if a Hall Sensor is present
//  DO NOT SHARE GROUND WITH A NEOPIXEL AND MECH SWITCH GROUND
//  DO NOT USE WITHIN A RTOS TASK
// -------------------------------------------------------------------
bool detectHallSensor(uint8_t pin) {
  // Drive pin LOW to completely drain residual charge to 0V
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delay(15);

  // Switch pin to Analog Input
  pinMode(pin, INPUT);
  delayMicroseconds(150); // Brief microsecond delay for active drivers to settle

  // Sample immediately before floating charge can build up
  uint32_t sum = 0;
  for (int i = 0; i < 5; i++) {
    sum += analogRead(pin);
    delayMicroseconds(500);
  }
  uint32_t avgRaw = sum / 5;

  if (Serial) Serial.printf("[STRUM SYSTEM] Pin %d Auto-Detect Raw Value: %u\n", pin, avgRaw);

  // RESULTS:
  // - Open / Floating Pin / Mech Switch -> Stays near 0V (Reads < 400 raw)
  // - Active Powered 49E Hall Sensor   -> Instantly re-charges to ~1.65V (~1500-2500 raw)
  if (avgRaw > 1200 && avgRaw < 3500) {
    return true; // Active Hall Sensor detected!
  }

  return false; // Fall back to Mechanical Switch mode
}

// -------------------------------------------------------------------
// Single pulse pattern (haptic)
// -------------------------------------------------------------------
void triggerStrumHaptic() {
  if (!hapticInitialized) return;

  if (xSemaphoreTake(xI2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    // Re-arm slot 0 to guarantee continuous re-triggering during fast strumming
    //haptic.setWaveform(0, 17); // Strong click
    //haptic.setWaveform(0, 47); // Strong longer buzz
    haptic.setWaveform(0, 1); 
    haptic.setWaveform(1, 0);

    haptic.go();
    xSemaphoreGive(xI2cMutex);
  }
}

// -------------------------------------------------------------------
// Triggers a specific ROM effect from the DRV2605 library
// 10 = Double Click 100%, 11 = Double Click 60%, 1 = Strong Click 100%
// -------------------------------------------------------------------
void playHapticEffect(uint8_t effect) {
  if (!hapticInitialized) return;

  if (xSemaphoreTake(xI2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    haptic.setWaveform(0, effect);
    haptic.setWaveform(1, 0); // End of sequence marker
    haptic.go();
    xSemaphoreGive(xI2cMutex);
  }
}

// Feeds a direct intensity value (0 to 127) for continuous vibration
void setHapticRTP(uint8_t intensity) {
  if (!hapticInitialized) return;

  if (xSemaphoreTake(xI2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    haptic.setRealtimeValue(intensity);
    xSemaphoreGive(xI2cMutex);
  }
}

// -------------------------------------------------------------------
// Battery level calculation
// -------------------------------------------------------------------
uint8_t getBatteryChargeLevel(uint32_t batteryMv, const BatteryProfile* curve, size_t numPoints) {
  // Multiply raw pin millivolts by 2 (for 100k/100k voltage divider)
  float volts = ((float)batteryMv * 2.0f) / 1000.0f;
  
  // Handle Out-of-Bounds (Fully Charged or Completely Dead)
  if (volts >= curve[0].voltage) {
    return curve[0].percentage;
  }
  if (volts <= curve[numPoints - 1].voltage) {
    return curve[numPoints - 1].percentage;
  }

  // Find the correct bracket and interpolate
  for (size_t i = 0; i < numPoints - 1; i++) {
    // Check if the current voltage falls between this point and the next point down
    if (volts <= curve[i].voltage && volts > curve[i+1].voltage) {
      
      // Calculate the total range between the two points
      float voltageRange = curve[i].voltage - curve[i+1].voltage;
      float percentRange = curve[i].percentage - curve[i+1].percentage;
      
      // Calculate how far along the curve we are from the lower point
      float voltageOffset = volts - curve[i+1].voltage;
      
      // Interpolate the exact percentage
      uint8_t percentage = curve[i+1].percentage + (uint8_t)((voltageOffset / voltageRange) * percentRange);
      
      if (Serial) Serial.printf("[BATTERY] Voltage: %.3f V | Bracket: [%.2f - %.2f] | Percentage: %u%%\n", volts, curve[i].voltage, curve[i+1].voltage, percentage);
      
      return percentage;
    }
  }

  return 0; // Fallback just in case
}

// -------------------------------------------------------------------
// Sets ADXL345 to Standby (0x00) or Measurement mode (0x08)
// -------------------------------------------------------------------
void setAdxl345PowerState(bool enable) {
  if (!accelInitialized) return;
  
  Wire.beginTransmission(0x53); // Default ADXL345 I2C address
  Wire.write(0x2D);            // POWER_CTL register
  Wire.write(enable ? 0x08 : 0x00); // 0x08 = Measure, 0x00 = Standby
  Wire.endTransmission();
}

// -------------------------------------------------------------------
// Setup ADXL345 Tap Feature
// -------------------------------------------------------------------
void setupAdxl345Tap(uint8_t tapThreshold) {
  Wire.beginTransmission(0x53);
  Wire.write(0x1D); // THRESH_TAP (62.5mg / LSB) -> 0x30 = ~3.0g force
  Wire.write(tapThreshold);
  Wire.endTransmission();

  Wire.beginTransmission(0x53);
  Wire.write(0x21); // DUR (625us / LSB) -> 0x10 = ~10ms duration
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(0x53);
  Wire.write(0x2A); // TAP_AXES (Enable Z axis tap detection)
  Wire.write(0x01); // Bit 0 = Z axis
  Wire.endTransmission();

  Wire.beginTransmission(0x53);
  Wire.write(0x2F); // INT_MAP (0 = INT1, 1 = INT2)
  Wire.write(0x00); // Route SINGLE_TAP interrupt to INT1
  Wire.endTransmission();

  Wire.beginTransmission(0x53);
  Wire.write(0x2E); // INT_ENABLE
  Wire.write(0x40); // Enable SINGLE_TAP interrupt (Bit 6)
  Wire.endTransmission();

  // Clear any pending interrupts generated during boot so the INT1 pin rests LOW
  Wire.beginTransmission(0x53);
  Wire.write(0x30); // INT_SOURCE
  Wire.endTransmission();
  Wire.requestFrom(0x53, 1);
  if (Wire.available()) Wire.read();
}

// -------------------------------------------------------------------
// Generate a unique firmware signature using compile time
// -------------------------------------------------------------------
String getFirmwareSignature() {
    // Combines version + date + time
    // Changes with every compile, even if FIRMWARE_VERSION doesn't change
    return String(VERSION) + "_" + 
           __DATE__ + "_" + 
           __TIME__;
}

// -------------------------------------------------------------------
// Determine if DRV2605 calibration is needed
// -------------------------------------------------------------------
bool checkDRV2605Calibration() {
    prefs.begin("ghero", false);
    
    // Check firmware signature
    String storedSig = prefs.getString("fw_sig", "");
    String currentSig = getFirmwareSignature();
    
    // Run calibration if new firmware or first ever boot -> run calibration
    if (storedSig != currentSig) {
      if (Serial) Serial.println("[DRV2605] New firmware detected or first boot - calibrating");
      if (Serial) Serial.println("[DRV2605] Current signature: " + currentSig);
      prefs.putString("fw_sig", currentSig);
      prefs.end();
      return true;
    }

    // Check if calibration data exists, if not -> run calibration
    if (!prefs.isKey("drv2605_cal")) {
      if (Serial) Serial.println("[DRV2605] No calibration data found - calibrating");
      prefs.end();
      return true;
    }

    prefs.end();
    return false;
}

// -------------------------------------------------------------------
// Load calibration data and apply to DRV2605 registers
// -------------------------------------------------------------------
bool loadDRV2605Calibration() {
    prefs.begin("ghero", false);
    
    if (!prefs.isKey("drv2605_cal")) {
      if (Serial) Serial.println("[DRV2605] No calibration data found");
      prefs.end();
      return false;
    }
    
    prefs.getBytes("drv2605_cal", drv2605CalibrationData, 3);
    prefs.end();
    
    // Apply calibration registers
    haptic.writeRegister8(DRV2605_REG_AUTOCALCOMP, drv2605CalibrationData[0]);
    haptic.writeRegister8(DRV2605_REG_AUTOCALEMP, drv2605CalibrationData[1]);

    // Apply ONLY the BEMF_GAIN bits (bits 1:0) in 0x1A (DRV2605_REG_FEEDBACK)
    uint8_t currentFeedback = haptic.readRegister8(DRV2605_REG_FEEDBACK);
    uint8_t restoredFeedback = (currentFeedback & 0xFC) | (drv2605CalibrationData[2] & 0x03);
    haptic.writeRegister8(DRV2605_REG_FEEDBACK, restoredFeedback);
    
    if (Serial) {
        Serial.println("[DRV2605] Calibration loaded:");
        Serial.printf("[DRV2605]   CAL1: 0x%02X, CAL2: 0x%02X, CAL3: 0x%02X\n",
                      drv2605CalibrationData[0],
                      drv2605CalibrationData[1],
                      drv2605CalibrationData[2]);
    }
    return true;
}

// -------------------------------------------------------------------
// Save calibration data from DRV2605 registers
// -------------------------------------------------------------------
void saveDRV2605Calibration() {
    // Read calibration registers
    drv2605CalibrationData[0] = haptic.readRegister8(DRV2605_REG_AUTOCALCOMP);
    drv2605CalibrationData[1] = haptic.readRegister8(DRV2605_REG_AUTOCALEMP);
    drv2605CalibrationData[2] = haptic.readRegister8(DRV2605_REG_FEEDBACK);
    
    prefs.begin("ghero", false);
    prefs.putBytes("drv2605_cal", drv2605CalibrationData, 3);
    prefs.end();
    
    if (Serial) {
        Serial.println("[DRV2605] Calibration saved:");
        Serial.printf("[DRV2605]   CAL1: 0x%02X, CAL2: 0x%02X, CAL3: 0x%02X\n",
                      drv2605CalibrationData[0],
                      drv2605CalibrationData[1],
                      drv2605CalibrationData[2]);
    }
}

// -------------------------------------------------------------------
// Initialize DRV2605 with smart calibration
// -------------------------------------------------------------------
void initHapticDriver() {
  if (haptic.begin(&Wire)) {
    haptic.useLRA();
    haptic.selectLibrary(6); // Library 6 = LRA Mode

    // Set 1.2V RMS Rated Voltage (Raw Value: 58 / 0x3A)
    haptic.writeRegister8(DRV2605_REG_RATEDV, 58);

    // Set 1.7V Peak Overdrive Clamp Voltage (Raw Value: 80 / 0x50)
    haptic.writeRegister8(DRV2605_REG_CLAMPV, 80);

    // Smart calibration
    if (checkDRV2605Calibration()) {
        // Run full calibration
        haptic.setMode(DRV2605_MODE_AUTOCAL);
        
        while (haptic.readRegister8(DRV2605_REG_GO) & 0x01) {    // Wait until calibration is complete (GO bit self-clears)
          delay(10);
        }

        haptic.setMode(DRV2605_MODE_INTTRIG);

        // Check bit 3 (DIAG_RESULT) in Register 0x00 -> 0 = Auto-calibration passed, 1 = Auto-calibration failed
        uint8_t status = haptic.readRegister8(0x00);

        if ((status & 0x08) == 0) {
            saveDRV2605Calibration();
        } else {
            Serial.println("[DRV2605] ERROR: Auto-calibration failed! Calibration not saved.");
        }
    } else {
        // Skip calibration, use stored data
        loadDRV2605Calibration();
    }

    hapticInitialized = true;
  }
  else {
    if (Serial) Serial.println("[MODULES] DRV2605L non-functional or disconnected.");
  }
}