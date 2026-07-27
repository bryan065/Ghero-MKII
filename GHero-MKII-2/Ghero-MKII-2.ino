/* -------------------------------------------------------------------------------------------------------------------------------
  ESP32 BLE Gamepad GUITAR Rockband Stratocaster (wii edition)
    
    # BUTTON SUMMARY
    __________________________________
    5 frets + solo frets (5 gpio) / No solo modifiers
    2 strum
    4 dpad
    2 start+select
    1 home
    1 toggle/function button
    1 whammy ADC
    1 ADXL345 i2c
    1 DRV2605 i2c (haptic motor)
    3 Neopixels

    # to-do
    __________________________________
      - Wireless Qi receiver
      - redo calibration button
        . only have hold 3s to enter calibration mode
      - RGB strum bar
        . Neopixel functions already finished
        . Waiting on neokey breakout boards
        . Different colors for the up/down strum on profile change
      - Waiting for DRV2605 for haptic feedback
        . Need to find a big enough haptic motor to simulate guitar strings being plucked
        . Need to implement dynamic feedback based on how hard/fast the strum is plucked
      - Waiting for 49E analog hall sensors
        . Will be soldered to the neokey breakout board
        . Hall sensor detection already implemented
      - Battery charging indicator in Windows
        . Since we can also attach a mouse device to the CompositeHID, we can use that to implement a charging indicator over BLE
        . ESP32-BLE-CompositeHID has not implemented the descriptors necessary for that, need to modify the library to add that function
        . Worth it?
  
  Library required:
    "ESP32-BLE-Composite" by Mystfit
    "Adafruit_ADXL345" by Adafruit
    "Adafruit_DRV2605" by Adafruit
    "Adafruit_NeoPixel" by Adafruit

  BOARD SPECS:
    Lilygo T-Energy S3
    LG 3000mah 18650

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
                                     3V3 | [ ]               [ ] | GND    / GND
                                   RESET | [ ]               [ ] | GPIO1  / RESET BUTTON
               -----------------/ GPIO37 | [ ]               [ ] | GPIO2  / HOME BUTTON / WAKE FROM SLEEP
                          START / GPIO35 | [ ]            TX [ ] | GPIO43 /-----------------
           SELECT / MENU / BACK / GPIO36 | [ ]            RX [ ] | GPIO44 /
                                / GPIO45 | [ ]               [ ] | GPIO42 /-----------------
                                / GPIO48 | [ ]               [ ] | GND    / GND
               -----------------/ GPIO47 | [ ]               [ ] | GPIO4  / PICKUP
                          RIGHT / GPIO21 | [ ]               [ ] | GPIO5  / NEOPIXEL
                             UP / GPIO14 | [ ]               [ ] | GPIO6  / STRUM UP
                           DOWN / GPIO13 | [ ]               [ ] | GPIO7  / STRUM DOWN
                           LEFT / GPIO12 | [ ]               [ ] | GPIO17 /-----------------
               -----------------/ GPIO11 | [ ]   HX6610S     [ ] | GPIO18
                            GND /    GND | [ ]    |          [ ] | GPIO0  /-----------------/ strapping pin / pull-up only
                                / GPIO10 | [x]----+ 50R      [ ] | GPIO8  / ORANGE FRET
                     WHAMMY ADC /  GPIO9 | [ ]               [ ] | GPIO41 / BLUE FRET
 pull-down only / strap ------- / GPIO46 | [ ]               [ ] | GPIO40 / YELLOW FRET
                                      NC | [ ]               [ ] | GPIO39 / RED FRET
                                      5V | [ ]               [ ] | GPIO38 / GREEN FRET
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

// DISABLE DEBUG FLAG BEFORE FINAL PRODUCTION USE
#define DEBUG

#include <Arduino.h>
#include <math.h>
#include "driver/rtc_io.h"
#include <Preferences.h>

#include <BleCompositeHID.h>
#include <XboxGamepadDevice.h>
#include <NimBLEDevice.h>
#include <Wire.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_DRV2605.h>
#include <Adafruit_NeoPixel.h>

// -------------------------------------------------------------------
// FUNCTION PROTOTYPES
// -------------------------------------------------------------------
void calibrateStrumZeroOnly();
void calibrateStrumFull();
void setAdxl345PowerState(bool enable);
bool detectHallSensor(uint8_t pin);
void triggerStrumHaptic();
void triggerDoubleHapticPulse();
uint8_t getBatteryChargeLevel(uint32_t batteryMv);

// -------------------------------------------------------------------
// Hardware Definitions
// -------------------------------------------------------------------

//USER PIN CONFIGURATION - CHANGE ONLY THESE VALUES
// ---------------------------------
#define ADC_PIN         3    // ADC pin on the lilygo T-Energy S3
#define CHARGING_PIN    10   // Wired to the open-drain CHRG pin on the HX charging controller
#define I2C_SDA_PIN     43   // i2c pin
#define I2C_SCL_PIN     44   // i2c
#define WHAMMY_ADC_PIN  9    // Whammy ADC pin
#define PICKUP_ADC_PIN  4    // Pickup ADC pin
#define NEOPIXEL_PIN    5    // neopixel pin
#define NUM_LEDS        3    // Number of neopixels. 1) Power status 2+3) strum backlight

#define WAKEUP_PIN      2    // Home button pin / RTC wakeup pin using ext0
#define STRUM_UP_PIN    6    // Strum up pin
#define STRUM_DOWN_PIN  7    // Strum down pin

#define BATTERY_TIMEOUT (10 * (60 * 1000))  // 10 minutes

// Default Sensitivity Scale (Percentage of maximum throw delta)
//   Ultra: 20% of max throw, High: 35%, Med: 50%, Low: 75%
const float PRESET_SCALES[2][5] = {
                                    { 0.20f, 0.35f, 0.50f, 0.75f, 1.0f }, // Up strum presets
                                    { 0.20f, 0.35f, 0.50f, 0.75f, 1.0f }  // Down strum presets
                                  }; 
// ---------------------------------
// Pickup range:
// 1 = 370
// 2 = 1240
// 3 = 2120
// 4 = 3015
// 5 = 4095

#define BATTERY_REPORT_INTERVAL (30 * 1000) // 30 seconds
#define SAMPLE_COUNT         10             // Sample count for battery detection
#define DRV2605_MODE_STANDBY 0x40           // 0x40 sets Bit 6 of Register 0x01, forcing the chip into low-power Standby mode

// -------------------------------------------------------------------
// BUTTON MAPS
// -------------------------------------------------------------------
// Helpers for xbox buttons
//   DO NOT CHANGE
#define HELPER_LT             0x21
#define HELPER_RT             0x22
#define HELPER_DPAD_UP        0x23
#define HELPER_DPAD_DOWN      0x24
#define HELPER_DPAD_LEFT      0x25
#define HELPER_DPAD_RIGHT     0x26
#define HELPER_FUNCTION       0xFFF

// Change the hall effect strum up/down mapping here
#define HALL_STRUM_UP         HELPER_DPAD_UP
#define HALL_STRUM_DOWN       HELPER_DPAD_DOWN

struct ButtonConfig {
  uint8_t  pin;
  uint16_t id;
  uint32_t debounceUs; // Per-button debounce delay in microseconds
};

// Button Maps
//   Change your pin and button assignment here
//
//   { GPIO_PIN, BUTTON_ID, DEBOUNCE_TIME }
const ButtonConfig BUTTON_MAP[] = {
  { 38,             HELPER_LT,            5000 }, // Green Fret
  { 39,             XBOX_BUTTON_LB,       5000 }, // Red Fret
  { 40,             XBOX_BUTTON_RB,       5000 }, // Yellow Fret
  { 41,             HELPER_RT,            5000 }, // Blue Fret
  { 8,              XBOX_BUTTON_A,        5000 }, // Orange Fret
  { 35,             XBOX_BUTTON_START,    5000 }, // Start
  { 36,             XBOX_BUTTON_SELECT,   5000 }, // Select / Menu
  { 1,              HELPER_FUNCTION,      5000 }, // Special Function key / Calibrate Hall Effect Strum
  { 14,             HELPER_DPAD_UP,       5000 }, // D-Pad Up
  { 13,             HELPER_DPAD_DOWN,     5000 }, // D-Pad Down
  { 12,             HELPER_DPAD_LEFT,     5000 }, // D-Pad Left
  { 21,             HELPER_DPAD_RIGHT,    5000 }, // D-Pad Right
  
// Don't change the pin definitions below, use the #define above
  { STRUM_UP_PIN,   HELPER_DPAD_UP,       8000 }, // Strum Up Switch
  { STRUM_DOWN_PIN, HELPER_DPAD_DOWN,     8000 }, // Strum Down Switch
  { WAKEUP_PIN,     XBOX_BUTTON_HOME,     5000 } // Home Button
};

constexpr size_t BUTTON_COUNT = sizeof(BUTTON_MAP) / sizeof(BUTTON_MAP[0]);

// Tracking button states and per-button timing
bool     lastDebouncedState[BUTTON_COUNT] = { false };
uint64_t lastStateChangeUs[BUTTON_COUNT] = { 0 };
// -------------------------------------------------------------------

// -------------------------------------------------------------------
// Preset Color Indicators For Activation Windows (Hall effect strum)
// -------------------------------------------------------------------
const uint32_t PRESET_COLORS[5] = {
  0xFF0000, // Preset 0: Red
  0x00FF00, // Preset 1: Green
  0x0000FF, // Preset 2: Blue
  0xFF00FF, // Preset 3: Magenta
  0xFFA500  // Preset 4: Orange
};

// Preferences Engine
Preferences prefs;

// Mutex to safely control BLE transmission across cores
SemaphoreHandle_t      xBleMutex = NULL;
SemaphoreHandle_t      xI2cMutex = NULL;
TaskHandle_t           rgbTaskHandle = NULL;
TaskHandle_t           hallEffectStrumTaskHandle = NULL;

// Battery config
volatile uint8_t       batteryLevel = 0;

// Shared core data
volatile unsigned long lastActivityTime = 0;  // Thread-safe variable for Core 1 (Sleep timer reset)
volatile int           sharedWhammyRaw = 0;   // Thread-safe variables for ADC's
volatile int           sharedPickupRaw = 0;
volatile uint32_t      sharedBatteryMv = 0;
volatile int           sharedStrumUpRaw = 0;
volatile int           sharedStrumDownRaw = 0;

// Hall Sensor Calibration Data
int                    strumUpZeroOffset = 0;
int                    strumDownZeroOffset = 0;
int                    strumUpMaxDelta = 800;   // Saved/Calibrated Max Displacement Default
int                    strumDownMaxDelta = 800; // Saved/Calibrated Max Displacement Default

// Dynamic indicator flag for RGB Task when user holds to re-calibrate
volatile bool          isCalibratingActive = false;

// Runtime Hall Sensor Detection
volatile bool          isHallEffectMode = false;

// Battery Charging Detection
volatile bool          isCharging = false;

// Lefty / Righty Auto-Detection Flag
bool                   isLeftHanded = false;

// Hall Effect Activation Threshold Index
volatile uint8_t       currentPresetIndex = 0;

bool accelInitialized  = false;
bool hapticInitialized = false;

// Whammy Parameters
constexpr uint32_t WHAMMY_INTERVAL_MS = 20;
constexpr int      RAW_MIN = 100;
constexpr int      RAW_MAX = 3900;
int                rawEma = -1;
int16_t            lastSentWhammy = -32768;

// Tilt Parameters
constexpr uint32_t TILT_INTERVAL_MS = 20;
int16_t            lastSentTilt = -32768;

// Deadzone size
constexpr int      HID_DEADBAND = 400;

// 1-second LED color feedback
volatile uint32_t  presetShowStartMs = 0;

// -------------------------------------------------------------------

// Initialize BLE gamepad
XboxGamepadDevice* gamepad;
BleCompositeHID compositeHID("Ghero MKII Guitar", "Harmonix", 100);

// Initialize ADXL345
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Initialize Haptic Driver
Adafruit_DRV2605 haptic;

// Initialize NeoPixels
Adafruit_NeoPixel strip(NUM_LEDS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// -------------------------------------------------------------------
// Handle button ID to gamepad inputs
// -------------------------------------------------------------------
inline void updateGamepadButton(uint16_t id, bool isPressed) {
  switch (id) {
    case HELPER_LT:         gamepad->setLeftTrigger(isPressed  ? XBOX_TRIGGER_MAX : XBOX_TRIGGER_MIN); break;
    case HELPER_RT:         gamepad->setRightTrigger(isPressed ? XBOX_TRIGGER_MAX : XBOX_TRIGGER_MIN); break;
    case HELPER_DPAD_UP:    gamepad->pressDPadDirectionFlag(isPressed ? NORTH : NONE); break;
    case HELPER_DPAD_DOWN:  gamepad->pressDPadDirectionFlag(isPressed ? SOUTH : NONE); break;
    case HELPER_DPAD_LEFT:  gamepad->pressDPadDirectionFlag(isPressed ? WEST  : NONE); break;
    case HELPER_DPAD_RIGHT: gamepad->pressDPadDirectionFlag(isPressed ? EAST  : NONE); break;
    case HELPER_FUNCTION:
//       if (isPressed && isHallEffectMode) {
//         currentPresetIndex = (currentPresetIndex + 1) % 4;
//         presetShowStartMs = millis(); // Trigger 1-second RGB indicator!

//         if (Serial) Serial.printf("[STRUM SYSTEM] Switched Strum Preset to Index: %d\n", currentPresetIndex);

//         // Save the new index to Flash
//         prefs.begin("ghero", false); // Open namespace in Read/Write mode
//         prefs.putUChar("strumPreset", currentPresetIndex);
//         prefs.end();
//       }
      break;
    default:
      if (isPressed) {  gamepad->press(id);   }
      else           {  gamepad->release(id); }
  }
}

// -------------------------------------------------------------------
// High Priority ADC Task (Core 1, Priority 4)
// -------------------------------------------------------------------
void adcTaskCore1(void *pvParameters) {
  for (;;) {
    // Continuous sampling of all analog hardware registers
    sharedWhammyRaw = analogRead(WHAMMY_ADC_PIN);
    sharedPickupRaw = analogRead(PICKUP_ADC_PIN);
    sharedBatteryMv = analogReadMilliVolts(ADC_PIN);

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

            triggerDoubleHapticPulse(); // Vibration feedback to confirm
          }
        }
      }
      // -----------------------------------

      // If any button state changed, send the combined report over BLE
      if (stateChanged) {
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
  // Variables for hall effect strum actuation
  static bool hallStrumUpState = false;
  static bool hallStrumDownState = false;

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

          vTaskDelete(hallEffectStrumTaskHandle);
          hallEffectStrumTaskHandle = NULL;
        }
      }
      else {
        // Dynamic scaling relative to saved max travel
        int upThresholdDelta = (int)(strumUpMaxDelta * PRESET_SCALES[1][currentPresetIndex]);
        int downThresholdDelta = (int)(strumDownMaxDelta * PRESET_SCALES[2][currentPresetIndex]);
        int hysteresis = 40;

        // Calculate absolute deviations relative to resting zero points
        int upDelta = abs(upRaw - strumUpZeroOffset);
        int downDelta = abs(downRaw - strumDownZeroOffset);

        // Evaluate logic with hysteresis
        bool newStrumUp = hallStrumUpState ? (upDelta > (upThresholdDelta - hysteresis)) 
                                          : (upDelta > upThresholdDelta);

        bool newStrumDown = hallStrumDownState ? (downDelta > (downThresholdDelta - hysteresis)) 
                                              : (downDelta > downThresholdDelta);

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

  for (;;) {
    // --- Calibration Flash (Highest Priority) ---
    if (isCalibratingActive) {
      for (int i = 0; i < NUM_LEDS; i++) {
        strip.setPixelColor(i, strip.Color(80, 0, 80)); // Magenta
      }
      strip.show();
      vTaskDelay(pdMS_TO_TICKS(50));
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
      float pitchAngle = atan2(ax, perpendicularMag) * 180.0f / M_PI;

      // Resting posture is typically ~0° to 15°.
      // Active Star Power activation window: 20° to 60° vertical pitch
      int16_t mappedTilt = 0;
      if (pitchAngle > 20.0f) {
        mappedTilt = (int16_t)map(constrain((int)pitchAngle, 20, 60), 20, 60, 0, 32767);
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
            presetShowStartMs = millis();

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
        vTaskDelay(pdMS_TO_TICKS(1)); // Yield briefly
      }
      float stdDev = sqrt(variance / SAMPLE_COUNT);

      // Pin voltage range for 1/2 divider is ~1600mV (3.2V) to ~2100mV (4.2V)
      bool isBatConnected = !(mean < 1200.0f || mean > 2300.0f || stdDev > 80.0f);

      uint8_t newLevel = 0;
      if (isBatConnected) {
        newLevel = getBatteryChargeLevel((uint32_t)mean);
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

// -------------------------------------------------------------------
// Setup
// -------------------------------------------------------------------
void setup() {
#ifdef DEBUG
  Serial.begin(115200);
  delay(1000); // Init delay
#endif

    // Initializes RGB
    strip.begin();
    strip.setBrightness(40); // Scale down maximum draw (0-255 scale)
    strip.show();

  // -------------------------------------------------------------------
  // SAFETY BOOT DELAY
  // Provides a 3-second recovery window on cold starts to allow
  // Arduino IDE / esptool to safely flash firmware in case of boot loops.
  // -------------------------------------------------------------------
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  if (Serial) Serial.printf("[SYSTEM] Wakeup Reason Enum: %d\n\n", wakeup_reason);
  
  // Only delay if it's a COLD START / POWER-ON (don't delay waking up from sleep!)
  if (wakeup_reason != ESP_SLEEP_WAKEUP_EXT0) {
    if (Serial) {
      Serial.println("");
      Serial.println("[SYSTEM] Upload recovery window active.");
    }

    delay(3000); // ~3 Second delay for Serial/CDC handshake during flash
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

  prefs.end(); // Close preference namespace

  if (Serial) Serial.printf("[PREFERENCES] Loaded Strum Config: Index %d | MaxDeltas Up:%d, Down:%d\n", currentPresetIndex, strumUpMaxDelta, strumDownMaxDelta);

#ifdef CHARGING_PIN
  pinMode(CHARGING_PIN, INPUT_PULLUP);
#endif

  xBleMutex = xSemaphoreCreateMutex();  // Create unified Mutex for thread-safe BLE updates
  xI2cMutex = xSemaphoreCreateMutex();  // Thread-safe i2c operations

  // Initialize GPIO 43 (SDA) and GPIO 44 (SCL) for ADXL345 Sensor
#if defined(I2C_SDA_PIN) && defined(I2C_SCL_PIN)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // --- ADXL345 Setup ---
  if (accel.begin()) {
    accel.setRange(ADXL345_RANGE_4_G);

    // Set sensor Output Data Rate (ODR) to 50 Hz (Register 0x2C = 0x09)
    Wire.beginTransmission(0x53);
    Wire.write(0x2C); // BW_RATE register
    Wire.write(0x09); // 50 Hz ODR
    Wire.endTransmission();

    accelInitialized = true;

    // Detect Handedness on Boot (Resting posture check)
    if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_EXT0) {
      delay(250); // Short settle delay
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
  else {
    if (Serial) Serial.println("[MODULES] ADXL345 non-functional or disconnected! Disabling tilt task.");
  }
  // --------------------

  // --- DRV2605L Setup ---
  if (haptic.begin(&Wire)) {
    haptic.setMode(DRV2605_MODE_INTTRIG); // Internal I2C Trigger Mode
    haptic.selectLibrary(6); // Library 6 = LRA Mode
    
    // Select the haptic effect:
    // Effect #1 = Strong Click (100%), #2 = Sharp Click (80%), #12 = Triple Click
    //haptic.setWaveform(0, 1); // Slot 0 = Sharp Click effect
    //haptic.setWaveform(1, 0); // Slot 1 = End of sequence marker

    hapticInitialized = true;
  }
  else {
    if (Serial) Serial.println("[MODULES] DRV2605L non-functional or disconnected.");
  }
#endif

  // ADC Attenuation Setup
#ifdef ARDUINO_ARCH_ESP32
  analogSetPinAttenuation(WHAMMY_ADC_PIN, ADC_11db);
  analogSetPinAttenuation(PICKUP_ADC_PIN, ADC_11db);
  analogSetPinAttenuation(STRUM_UP_PIN  , ADC_11db);
  analogSetPinAttenuation(STRUM_DOWN_PIN, ADC_11db);
  analogSetPinAttenuation(ADC_PIN       , ADC_11db);
  analogReadResolution(12);
#endif

  // Auto-detect strum mechanism BEFORE configuring button pull-ups
  isHallEffectMode |= detectHallSensor(STRUM_UP_PIN);
  isHallEffectMode |= detectHallSensor(STRUM_DOWN_PIN);

  if (Serial) Serial.printf("[STRUM SYSTEM] Strum Hardware: %s\n", isHallEffectMode ? "HALL EFFECT" : "MECHANICAL SWITCH");

  // Whammy and Pickup ADC setup
  pinMode(WHAMMY_ADC_PIN, INPUT);
  pinMode(PICKUP_ADC_PIN, INPUT);

  if (isHallEffectMode) {
    pinMode(STRUM_UP_PIN, INPUT);
    pinMode(STRUM_DOWN_PIN, INPUT);
  }

  // Configure non-strum physical button pins as inputs with pull-ups
  for (size_t i = 0; i < BUTTON_COUNT; i++) {
    if (isHallEffectMode && (BUTTON_MAP[i].pin == STRUM_UP_PIN || BUTTON_MAP[i].pin == STRUM_DOWN_PIN)) {
      continue; // Skip pull-up config on Hall analog pins
    }
    pinMode(BUTTON_MAP[i].pin, INPUT_PULLUP);
  }

  // --- CORE 1 TASKS ---
  // ADC Task (Priority 4)
  xTaskCreatePinnedToCore(
    adcTaskCore1,       // Single ADC reader task
    "AdcTaskCore1",     // Task name
    4096,               // Stack size
    NULL,               // Parameter
    4,                  // Slightly lower priority than main button handler
    NULL,               // Task handle
    1                   // Core 1
  );

  // High-Priority Button Task (Priority 5)
  xTaskCreatePinnedToCore(
    buttonTaskCore1,    // Button & strum processing task
    "ButtonTaskCore1",  // Task name
    8192,
    NULL,
    5,                  // Highest Priority
    NULL,
    1                   // Core 1
  );

  // High-Priority Hall Effect Strum Task (Priority 5)
  if (isHallEffectMode) {
    xTaskCreatePinnedToCore(
      hallEffectStrumTaskCore1,    // Button & strum processing task
      "hallEffectStrumTaskCore1",  // Task name
      8192,
      NULL,
      5,
      &hallEffectStrumTaskHandle,
      1
    );
  }

  // --- CORE 0 TASKS ---
  // Battery Monitor Task (Priority 1)
#ifdef ADC_PIN
  xTaskCreatePinnedToCore(
    batteryTaskCore0,     // Process battery level & detection
    "BatteryTaskCore0",   // Task name
    4096,                 // Stack size
    NULL,                 // Parameter
    1,                    // Priority (Low)
    NULL,                 // Task handle
    0                     // Core ID (0)
  );
#endif

  // Whammy Task (Priority 1)
  xTaskCreatePinnedToCore(
    whammyTaskCore0,      // Whammy polling task
    "WhammyTaskCore0",    
    4096,                 
    NULL,                 
    1,                    
    NULL,                 
    0                     
  );

  // ADXL345 Tilt Task (Priority 1)
  //  note: don't create task if adxl is malfunctioning
  if (accelInitialized) {
    xTaskCreatePinnedToCore(
      tiltTaskCore0,      // Tilt sensor polling task
      "TiltTaskCore0",
      4096,
      NULL,
      1,                  // Low Priority (matches Whammy)
      NULL,
      0                   // Core 0
    );
  }

  // NeoPixel RGB Task (Priority 1)
  xTaskCreatePinnedToCore(
    rgbTaskCore0,         // NeoPixel animation task
    "RgbTaskCore0",
    3072,
    NULL,
    1,                    // Low Priority
    &rgbTaskHandle,       // Handle
    0                     // Core 0
  );

  // Calibrate zero offset of hall sensors
  if (isHallEffectMode) {
    delay(100); // Brief pause to ensure the ADC task has taken its first pass
    calibrateStrumZeroOnly();
  }

  // Setup controller
  XboxSeriesXControllerDeviceConfiguration* config = new XboxSeriesXControllerDeviceConfiguration();
  BLEHostConfiguration hostConfig = config->getIdealHostConfiguration();
  hostConfig.setHidType(HID_GAMEPAD);
  gamepad = new XboxGamepadDevice(config);

  compositeHID.addDevice(gamepad);
  compositeHID.begin(hostConfig);

  // Set TX power to lower levels since we'll be in LoS of the PC (right...?)
  // ---
  //    Negative db: ESP_PWR_LVL_N12, ESP_PWR_LVL_N9, ESP_PWR_LVL_N6, ESP_PWR_LVL_N3
  //                 ESP_PWR_LVL_N0
  //    Positive db: ESP_PWR_LVL_P3, ESP_PWR_LVL_P6, ESP_PWR_LVL_P9
  //NimBLEDevice::setPower(ESP_PWR_LVL_P3);

  // Setup wake pin using ext0
  esp_sleep_enable_ext0_wakeup((gpio_num_t)WAKEUP_PIN, 0);

  // Start timeout timer
  lastActivityTime = millis();

  // Trigger 1-second color feedback for the loaded preset on boot
  presetShowStartMs = millis();
}

// -------------------------------------------------------------------
// Main Loop is only running sleep functions on low priority
// -------------------------------------------------------------------
void loop() {
  // Check inactivity timeout regardless of BLE connection state
  if (millis() - lastActivityTime > BATTERY_TIMEOUT) {
    if (Serial) Serial.println("[POWER] Inactivity timeout reached. Entering Deep Sleep...");
    Serial.flush();

    // Stops RGBtask and clears neopixels
    if (rgbTaskHandle != NULL) {
      vTaskDelete(rgbTaskHandle);
      rgbTaskHandle = NULL;
    }
    strip.clear();
    strip.show();
    delay(20);

    // Put I2C Peripherals to Sleep
    setAdxl345PowerState(false); // Standby mode (~0.1 uA)
    haptic.setMode(DRV2605_MODE_STANDBY); // DRV2605 into standby (~1.5 uA)

    // Explicitly Disable Bluetooth Radio
    btStop();

    // Configure RTC Wakeup Pin
    rtc_gpio_pullup_en((gpio_num_t)WAKEUP_PIN);
    rtc_gpio_pulldown_dis((gpio_num_t)WAKEUP_PIN);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)WAKEUP_PIN, 0); // Wake on LOW press

    // Enter Deep Sleep
    if (Serial) Serial.end();
    pinMode(NEOPIXEL_PIN, INPUT); // Disable active output push-pull state
    gpio_deep_sleep_hold_en(); // Lock pin states during sleep
    esp_deep_sleep_start();
  }

  vTaskDelay(pdMS_TO_TICKS(1000));
}

// -------------------------------------------------------------------
// HELPER FUNCTIONS
// -------------------------------------------------------------------

// Calibrate hall sensor 0-values for 1 second
// -------------------------------------------------------------------
void calibrateStrumZeroOnly() {
  if (!isHallEffectMode) return;

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

// Calibrate hall sensor (Full calibration)
// -------------------------------------------------------------------
void calibrateStrumFull() {
  if (!isHallEffectMode) return;

  // Zero-offset resting point (1 second)
  calibrateStrumZeroOnly();

  // Maximum travel range capture window (5 seconds)
  uint32_t windowStart = millis();
  int maxObservedUpDelta = 0;
  int maxObservedDownDelta = 0;

  while (millis() - windowStart < 5000) {
    int curUpRaw = sharedStrumUpRaw;
    int curDownRaw = sharedStrumDownRaw;

    int curUpDelta = abs(curUpRaw - strumUpZeroOffset);
    int curDownDelta = abs(curDownRaw - strumDownZeroOffset);

    if (curUpDelta > maxObservedUpDelta)     maxObservedUpDelta = curUpDelta;
    if (curDownDelta > maxObservedDownDelta) maxObservedDownDelta = curDownDelta;

    // Don't sleep
    lastActivityTime = millis();

    vTaskDelay(pdMS_TO_TICKS(2));
  }

  // Require a minimum noise-floor delta (200 raw steps) to confirm actual strumming took place
  bool updated = false;
  
  if (maxObservedUpDelta >= 200) {
    strumUpMaxDelta = maxObservedUpDelta;
    updated = true;
  }
  
  if (maxObservedDownDelta >= 200) {
    strumDownMaxDelta = maxObservedDownDelta;
    updated = true;
  }

  // Save to persistent storage if valid strumming was performed
  if (updated) {
    prefs.begin("ghero", false);
    prefs.putInt("strumUpMax", strumUpMaxDelta);
    prefs.putInt("strumDownMax", strumDownMaxDelta);
    prefs.end();

    if (Serial) Serial.printf("[STRUM SYSTEM] New Strum Limits Saved! Up Max: %d | Down Max: %d\n", strumUpMaxDelta, strumDownMaxDelta);
  }
  else {
    if (Serial) Serial.println("[STRUM SYSTEM] No strumming detected during 5s window. Retaining prior thresholds.");
  }
}

// Detects if a Hall Sensor is present
//  DO NOT USE WITHIN A RTOS TASK
// -------------------------------------------------------------------
bool detectHallSensor(uint8_t pin) {
  // Step 1: Drive pin LOW to completely drain residual charge to 0V
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delay(15);

  // Step 2: Switch pin to Analog Input
  pinMode(pin, INPUT);
  delayMicroseconds(100); // Brief microsecond delay for active drivers to settle

  // Step 3: Sample immediately before floating charge can build up
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
  if (avgRaw > 1000) {
    return true; // Active Hall Sensor detected!
  }

  return false; // Fall back to Mechanical Switch mode
}

// Single pulse pattern (haptic)
// -------------------------------------------------------------------
inline void triggerStrumHaptic() {
  if (!hapticInitialized) return;

  if (xSemaphoreTake(xI2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    // Re-arm slot 0 to guarantee continuous re-triggering during fast strumming
    haptic.setWaveform(0, 1); 
    haptic.setWaveform(1, 0);

    haptic.go();
    xSemaphoreGive(xI2cMutex);
  }
}

// Double pulse pattern to confirm calibration finished
// -------------------------------------------------------------------
void triggerDoubleHapticPulse() {
  if (!hapticInitialized) return;

  if (xSemaphoreTake(xI2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    haptic.setWaveform(0, 12); // Triple/Double Click sequence
    haptic.setWaveform(1, 0);
    haptic.go();
    xSemaphoreGive(xI2cMutex);
  }
}

// Battery level calculation
// -------------------------------------------------------------------
uint8_t getBatteryChargeLevel(uint32_t batteryMv) {
  // Multiply raw pin millivolts by 2 (for 100k/100k voltage divider on T-Energy S3)
  float volts = ((float)batteryMv * 2.0f) / 1000.0f;

  uint8_t percentage = 0;

  if (volts >= 4.15f) {
    percentage = 100;
  } else if (volts <= 3.20f) {
    percentage = 0;
  } else if (volts < 3.70f) {
    // 0% to 10% range (3.2V to 3.7V)
    percentage = (uint8_t)(((volts - 3.20f) / 0.50f) * 10.0f);
  } else if (volts <= 4.10f) {
    // 10% to 90% range (3.7V to 4.1V)
    percentage = 10 + (uint8_t)(((volts - 3.70f) / 0.40f) * 80.0f);
  } else {
    // 90% to 100% range (4.1V to 4.2V)
    percentage = 90 + (uint8_t)(((volts - 4.10f) / 0.10f) * 10.0f);
  }

  if (Serial) Serial.printf("[BATTERY] Raw Pin: %u mV | Calc Voltage: %.3f V | Percentage: %u%%\n", batteryMv, volts, percentage);

  return percentage;
}

// Sets ADXL345 to Standby (0x00) or Measurement mode (0x08)
// -------------------------------------------------------------------
void setAdxl345PowerState(bool enable) {
  if (!accelInitialized) return;
  
  Wire.beginTransmission(0x53); // Default ADXL345 I2C address
  Wire.write(0x2D);            // POWER_CTL register
  Wire.write(enable ? 0x08 : 0x00); // 0x08 = Measure, 0x00 = Standby
  Wire.endTransmission();
}