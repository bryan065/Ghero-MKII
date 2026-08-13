#include <Arduino.h>
#include "driver/rtc_io.h"
#include "Config.h"
#include "Globals.h"
#include "Hardware.h"
#include "Tasks.h"

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#endif

  // -------------------------------------------------------------------
  // SAFETY BOOT DELAY
  // Provides a 3-second recovery window on cold starts to allow
  // Arduino IDE / esptool to safely flash firmware in case of boot loops.
  // -------------------------------------------------------------------
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  if (Serial) Serial.printf("[SYSTEM] Wakeup Reason Enum: %d\n", wakeup_reason);
  
  // Only delay if it's a COLD START / POWER-ON (don't delay waking up from sleep!)
  if (wakeup_reason != ESP_SLEEP_WAKEUP_EXT0) {
    delay(500);

    if (Serial) {
      Serial.println("");
      Serial.println("[SYSTEM] Upload recovery window active.");
      delay(2500); // ~3 Second delay for Serial/CDC handshake during flash
    }
    
    if (Serial) Serial.println("[SYSTEM] Starting System...\n");
  }
  else {
    // deinit wakeup pin so we can use it for GPIO again
    rtc_gpio_deinit((gpio_num_t)WAKEUP_PIN);

    if (Serial) Serial.println("[SYSTEM] Waking from sleep!\n");
  }
  // -------------------------------------------------------------------

  // Load preferences and prevent preset OOB
  prefs.begin("ghero", true);
  currentPresetIndex = prefs.getUChar("strumPreset", 3);
  if (currentPresetIndex >= 5) currentPresetIndex = 0;

  // Load saved strum calibration ranges (default to 800 if not found)
  strumUpMaxDelta = prefs.getInt("strumUpMax", 800);
  strumDownMaxDelta = prefs.getInt("strumDownMax", 800);

  // Load custom index 0 multipliers (default to 20% / 0.20f if not found)
  PRESET_SCALES[0][0] = prefs.getFloat("custUpMult", 0.20f);
  PRESET_SCALES[1][0] = prefs.getFloat("custDwnMult", 0.20f);

  prefs.end();

  if (Serial) Serial.printf("[PREFERENCES] Loaded Strum Config: Index %d | MaxDeltas Up:%d, Down:%d\n", currentPresetIndex, strumUpMaxDelta, strumDownMaxDelta);

#ifdef CHARGING_PIN
  pinMode(CHARGING_PIN, INPUT_PULLUP);
#endif

  xBleMutex = xSemaphoreCreateMutex();
  xI2cMutex = xSemaphoreCreateMutex();
  xTapSemaphore = xSemaphoreCreateCounting(10, 0);

  // Initialize GPIO 43 (SDA) and GPIO 44 (SCL) for ADXL345 Sensor
#if defined(I2C_SDA_PIN) && defined(I2C_SCL_PIN)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // ADXL345 Setup
  if (accel.begin()) {
    accel.setRange(ADXL345_RANGE_4_G);
    accel.setDataRate(ADXL345_DATARATE_50_HZ);

    if (Serial) Serial.printf("[SYSTEM] ADXL Data Rate: %d\n", accel.getDataRate());

    setupAdxl345Tap(TAP_SENSITIVITY);
    accelInitialized = true;

    // Detect Handedness on Boot (Resting posture check)
    if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_EXT0) {
      delay(250);
      sensors_event_t event;
      accel.getEvent(&event);

      // Threshold: 2.0 m/s^2 (~0.2g) deadzone around zero
      const float UPRIGHT_X_THRESHOLD = 2.0f;

      // If X-axis acceleration is negative, neck is pointing right (Left-Handed mode)
      if (event.acceleration.x < -UPRIGHT_X_THRESHOLD) {
        isLeftHanded = true;
      }

      if (Serial) Serial.printf("[ACCEL] Boot orientation: %s-Handed (Ax = %.2f)\n", isLeftHanded ? "LEFT" : "RIGHT", event.acceleration.x);
    }
  }

  // DRV2605L Setup
  initHapticDriver();
#endif

  // ADC Attenuation Setup
#ifdef ARDUINO_ARCH_ESP32

#ifdef WHAMMY_ADC_PIN
  analogSetPinAttenuation(WHAMMY_ADC_PIN,  ADC_11db);
#endif

#ifdef PICKUP_ADC_PIN
  analogSetPinAttenuation(PICKUP_ADC_PIN,  ADC_11db);
#endif

#ifdef BATTERY_ADC_PIN
  analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);
#endif

  analogSetPinAttenuation(STRUM_UP_PIN  ,  ADC_11db);
  analogSetPinAttenuation(STRUM_DOWN_PIN,  ADC_11db);
  analogReadResolution(12);

#endif

  // Auto-detect strum mechanism BEFORE configuring button pull-ups
  isHallEffectMode |= detectHallSensor(STRUM_UP_PIN);
  isHallEffectMode |= detectHallSensor(STRUM_DOWN_PIN);

  if (Serial) Serial.printf("[STRUM SYSTEM] Strum Hardware: %s\n", isHallEffectMode ? "HALL EFFECT" : "MECHANICAL SWITCH");

  // Setup Pickguard Tap Interrupt
  #ifdef TAP_INT_PIN
  if (accelInitialized) {
    pinMode(TAP_INT_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(TAP_INT_PIN), adxlTapISR, RISING);
  }
  #endif

  if (isHallEffectMode) {
    pinMode(STRUM_UP_PIN, INPUT);
    pinMode(STRUM_DOWN_PIN, INPUT);
  }

  // Configure non-strum physical button pins as inputs with pull-ups
  for (size_t i = 0; i < BUTTON_COUNT; i++) {
    if (isHallEffectMode && (BUTTON_MAP[i].pin == STRUM_UP_PIN || BUTTON_MAP[i].pin == STRUM_DOWN_PIN)) {
      continue; 
    }
    pinMode(BUTTON_MAP[i].pin, INPUT_PULLUP);
  }

  // Start RTOS Tasks
  //   [task], [name], [stack size], [parameter], [priority], [handle], [core]
  xTaskCreatePinnedToCore(buttonTaskCore1,  "ButtonTaskCore1",  8192, NULL, 5, NULL,            1);

#if defined(WHAMMY_ADC_PIN) || defined(PICKUP_ADC_PIN) || defined(BATTERY_ADC_PIN)
  xTaskCreatePinnedToCore(adcTaskCore1,     "AdcTaskCore1",     4096, NULL, 4, NULL,            1);
#endif

#ifdef NEOPIXEL_PIN
  xTaskCreatePinnedToCore(rgbTaskCore0,     "RgbTaskCore0",     3072, NULL, 1, &rgbTaskHandle,  0);
#endif

#ifdef WHAMMY_ADC_PIN
  xTaskCreatePinnedToCore(whammyTaskCore0,  "WhammyTaskCore0",  4096, NULL, 1, NULL,            0);
#endif
  
#ifdef BATTERY_ADC_PIN
  xTaskCreatePinnedToCore(batteryTaskCore0, "BatteryTaskCore0", 4096, NULL, 1, NULL,            0);
#endif
  
  if (accelInitialized) xTaskCreatePinnedToCore(tiltTaskCore0,            "TiltTaskCore0",  4096, NULL, 1, NULL,                       0);
  if (isHallEffectMode) xTaskCreatePinnedToCore(hallEffectStrumTaskCore1, "hallStrumTask",  8192, NULL, 5, &hallEffectStrumTaskHandle, 1);
  if (accelInitialized) xTaskCreatePinnedToCore(tapTaskCore1,             "TapTaskCore1",   3072, NULL, 1, NULL,                       1);
  
  // Calibrate zero offset of hall sensors
  if (isHallEffectMode) {
    delay(100); 
    calibrateStrumZeroOnly();
  }

  // Setup controller
  XboxSeriesXControllerDeviceConfiguration* config = new XboxSeriesXControllerDeviceConfiguration();
  BLEHostConfiguration hostConfig = config->getIdealHostConfiguration();
  hostConfig.setHidType(HID_GAMEPAD);
  hostConfig.setSoftwareRevision(VERSION);
  gamepad = new XboxGamepadDevice(config);

  compositeHID.addDevice(gamepad);
  compositeHID.begin(hostConfig);

  esp_sleep_enable_ext0_wakeup((gpio_num_t)WAKEUP_PIN, 0);

  lastActivityTime = millis();
  presetShowStartMs = millis();
}

void loop() {
  // Check inactivity timeout regardless of BLE connection state
  if (millis() - lastActivityTime > BATTERY_TIMEOUT) {
    if (Serial) Serial.println("[POWER] Timeout reached. Deep Sleep.");
    Serial.flush();
    
    // Stops RGBtask and clears neopixels
#ifdef NEOPIXEL_PIN
    if (rgbTaskHandle != NULL) {
      vTaskDelete(rgbTaskHandle);
      rgbTaskHandle = NULL;
    }
    strip.clear();
    strip.show();
    delay(20);

    pinMode(NEOPIXEL_PIN, INPUT);
#endif

    // Put I2C Peripherals to Sleep
    setAdxl345PowerState(false);           // Standby mode (~0.1 uA)
    haptic.setMode(DRV2605_MODE_STANDBY);  // DRV2605 into standby (~1.5 uA)
    btStop();

    // Configure RTC Wakeup Pin
    rtc_gpio_pullup_en((gpio_num_t)WAKEUP_PIN);
    rtc_gpio_pulldown_dis((gpio_num_t)WAKEUP_PIN);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)WAKEUP_PIN, 0);

    // Enter Deep Sleep
    if (Serial) Serial.end();
    gpio_deep_sleep_hold_en(); 
    esp_deep_sleep_start();
  }
  vTaskDelay(pdMS_TO_TICKS(1000));
}