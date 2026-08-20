/* -------------------------------------------------------------------------------------------------------------------------------
  ESP32 BLE Gamepad GUITAR Rockband Stratocaster (wii edition)
    
    # BUTTON SUMMARY
    __________________________________
    5 frets + solo frets (5 gpio) / No solo modifiers
    2 strum + hall effect sensors (same pin) -> 49E sensors
    4 dpad
    2 start+select
    1 home
    1 function button
    1 whammy ADC
    1 pickup ADC
    1 ADXL345 i2c -> i2c
    1 DRV2605 i2c (haptic motor) - > i2c
    3 Neopixels

    # to-do
    __________________________________
      - feat: Battery charging indicator in Windows
        . Since we can also attach a mouse device to the CompositeHID, we can use that to implement a charging indicator over BLE
        . ESP32-BLE-CompositeHID has not implemented the descriptors necessary for that, need to modify the library to add that function
        . Worth it?
      - feat: Haptics - > dual motors
        . Explore different haptic controller + higher voltage motor for main body
        . DRV2605 for guitar neck
        - ESP32-S3 has two i2c buses. We can use a different bus for the secondary DRV2605
      - feat: Add a "rake" strum mode - > 1/1 or 2/2 strum bar
        . When RAKE is defined and pickup encoder is set to a configurable profile -> every strum will send an opposite strum shortly after (70ms?). Allows for double strumming or "rake" strumming
      - fix: Replace defective neokey
        . One RGB is flickering, need to find if it's wiring or neopixel issue
      - fix: possibly update tasks and move bleTask to it's own thread/task so only one task is sending BLE events. May improve or reduce button latency
  
  Library required:
    "ESP32-BLE-CompositeHID" by Mystfit
    "Adafruit_ADXL345" by Adafruit
    "Adafruit_DRV2605" by Adafruit
    "Adafruit_NeoPixel" by Adafruit
    "bounce2"

  HARDWARE SPECS:
    . Lilygo T-Energy S3
    . LG 3000mah 18650
    . Adafruit Neokey PCB
    . 49E Hall effect sensors
    . DRV2605
    . ADXL345

  Board settings (changed from defaults):
    USB CDC           = enabled
    CPU freqency      = 80Mhz
    Flash size        = 16MB (128Mb)
    Partition scheme  = 16M Flash (3M/9.9M)
    PSRAM             = disabled

  WIRING:
  (each: 1 leg -> GPIO, opposite leg -> GND)

                                         +-----------------------+
                                         | O      | BAT |      O |
                                         |        |GPIO3|        |
 VCC -> i2c, neopixel, hall sensor / 3V3 | [ ]               [ ] | GND    / GND
                                   RESET | [ ]               [ ] | GPIO1  / RESET BUTTON
               -----------------/ GPIO37 | [ ]               [ ] | GPIO2  / HOME BUTTON / WAKE FROM SLEEP
                          START / GPIO35 | [ ]      I2C / TX [ ] | GPIO43 /-----------------
           SELECT / MENU / BACK / GPIO36 | [ ]      I2C / RX [ ] | GPIO44 /
           PICKGUARD TAP - SLAP / GPIO45 | [ ]               [ ] | GPIO42 /-----------------
                                / GPIO48 | [ ]               [ ] | GND    / GND
               -----------------/ GPIO47 | [ ]               [ ] | GPIO4  / PICKUP ADC
                          RIGHT / GPIO21 | [ ]               [ ] | GPIO5  / NEOPIXEL
                             UP / GPIO14 | [ ]               [ ] | GPIO6  / STRUM UP
                           DOWN / GPIO13 | [ ]               [ ] | GPIO7  / STRUM DOWN
                           LEFT / GPIO12 | [ ]               [ ] | GPIO17 /-----------------
               -----------------/ GPIO11 | [ ]   HX6610S     [ ] | GPIO18
                            GND /    GND | [ ]    |          [ ] | GPIO0  /-----------------/ strapping pin / pull-up only
            BATTERY CHARGE GPIO / GPIO10 | [x]----+ 50R      [ ] | GPIO8  / ORANGE FRET
                     WHAMMY ADC /  GPIO9 | [ ]               [ ] | GPIO41 / BLUE FRET
 pull-down only / strap ------- / GPIO46 | [ ]               [ ] | GPIO40 / YELLOW FRET
                                      NC | [ ]               [ ] | GPIO39 / RED FRET
      QI WIRELESS -> SCHOTTKY DIODE / 5V | [ ]               [ ] | GPIO38 / GREEN FRET
                                      NC | [ ]               [ ] | NC
                                         |                       |
                                         | [USB]  [QWIIC]        |
                                         | [USB]                 |
                                         | O                   O |
                                         +-----------------------+

  NOTE:
    GPIO10 is bridged to pin 6 of the HX6610S with a _SMALL RESISTOR_ for charging detection. When battery is charging, the red LED is lit up and pin 6 on the HX6610S is pulled low.
    Make sure to set GPIO10 to internal pullup.

    Additionally, you can do the same thing to pin 7 on the HX6610S for standby indication. (USB power but not charging?)

    GPIO43 / GPIO44 are the RX/TX pins on the QWIIC connector for an i2c tilt sensor + more


  All digital pins use internal pull-ups — no external resistors needed.

  NOTE:
  ======
  Only RTC IO can be used as a source for external wake
  source. They are pins: 0,2,4,12-15,25-27,32-39.

  ADC pin is for a Lilygo T-Energy S3 battery voltage divider (GPIO03)
------------------------------------------------------------------------------------------------------------------------------- */

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
  if (!buttonsInitialized) {
    buttonsInitialized = initButtonBounce();
  }

  // Start RTOS Tasks
  //   [task], [name], [stack size], [parameter], [priority], [handle], [core]
  xTaskCreatePinnedToCore(buttonTask,  "ButtonTask",  8192, NULL, 5, NULL,            1);

#if defined(WHAMMY_ADC_PIN) || defined(PICKUP_ADC_PIN) || defined(BATTERY_ADC_PIN)
  xTaskCreatePinnedToCore(adcTask,     "AdcTask",     4096, NULL, 4, NULL,            1);
#endif

#ifdef NEOPIXEL_PIN
  xTaskCreatePinnedToCore(rgbTask,     "RgbTask",     4096, NULL, 4, &rgbTaskHandle,  1);
#endif

#ifdef WHAMMY_ADC_PIN
  xTaskCreatePinnedToCore(whammyTask,  "WhammyTask",  4096, NULL, 2, NULL,            0);
#endif
  
#ifdef BATTERY_ADC_PIN
  xTaskCreatePinnedToCore(batteryTask, "BatteryTask", 4096, NULL, 1, NULL,            0);
#endif
  
  if (accelInitialized) xTaskCreatePinnedToCore(tiltTask,            "TiltTask",  4096, NULL, 1, NULL,                            0);
  if (isHallEffectMode) xTaskCreatePinnedToCore(hallEffectStrumTask, "hallStrumTask",  8192, NULL, 5, &hallEffectStrumTaskHandle, 1);
  if (accelInitialized) xTaskCreatePinnedToCore(tapTask,             "TapTask",   3072, NULL, 2, NULL,                            0);
  
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