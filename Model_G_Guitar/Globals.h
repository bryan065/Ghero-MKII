#pragma once
#include "Config.h"
#include <Bounce2.h>
#include <Preferences.h>
#include <BleCompositeHID.h>
#include <XboxGamepadDevice.h>
#include <NimBLEDevice.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_DRV2605.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>

// Fixed Configuration Options
#define BATTERY_REPORT_INTERVAL (30 * 1000) // 30 Seconds
#define SAMPLE_COUNT            10
#define DRV2605_MODE_STANDBY    0x40

// Pickguard Tap Sensivity options
#define TAP_LOW                 0x50  // ~5.0g force (least sensitive, highest force required)
#define TAP_MED                 0x40  // ~4.0g force (medium sensitivity)
#define TAP_HIGH                0x30  // ~3.0g force (high sensitivity)
#define TAP_ULTRA               0x20  // ~2.0g force (ultra sensitive)

// Structs & Enums
struct BatteryProfile {
  float voltage;
  uint8_t percentage;
};

enum CalibState {
  CAL_NONE, CAL_ZERO, CAL_FULL_STRUM, CAL_SUCCESS_PULSE,
  CAL_UP_HOLD, CAL_DOWN_HOLD, CAL_DONE_PULSES
};

// Constants and Arrays
constexpr uint32_t WHAMMY_INTERVAL_MS = 20;
constexpr uint32_t TILT_INTERVAL_MS = 20;
constexpr int RAW_MIN = 100;
constexpr int RAW_MAX = 3900;
constexpr int HID_DEADBAND = 400;

extern const BatteryProfile generic[];
extern const BatteryProfile lg_hg2[];
extern float PRESET_SCALES[2][5];
extern const uint32_t PRESET_COLORS[5];

// Battery Profiles
inline const BatteryProfile generic[8] = {
  {4.20f, 100}, {4.10f, 90}, {4.00f, 80}, {3.90f, 60}, 
  {3.80f, 40},  {3.70f, 20}, {3.60f, 10}, {3.30f, 0}
};

inline const BatteryProfile lg_hg2[7] = {
  {4.15f, 100}, {4.05f, 95}, {3.90f, 80}, {3.70f, 40}, 
  {3.50f, 15},  {3.30f, 5},  {3.20f, 0}
};

// RTOS & Hardware Objects
extern Preferences prefs;
extern SemaphoreHandle_t xBleMutex;
extern SemaphoreHandle_t xI2cMutex;
extern SemaphoreHandle_t xTapSemaphore;
extern TaskHandle_t      rgbTaskHandle;
extern TaskHandle_t      hallEffectStrumTaskHandle;

extern XboxGamepadDevice* gamepad;
extern BleCompositeHID compositeHID;
extern Adafruit_ADXL345_Unified accel;
extern Adafruit_DRV2605 haptic;
extern Adafruit_NeoPixel strip;

// State Tracking & Internals
extern bool     lastDebouncedState[BUTTON_COUNT];
extern uint64_t lastStateChangeUs[BUTTON_COUNT];

extern volatile CalibState calState;
extern volatile uint32_t   calPhaseStartMs;   
extern volatile bool       isCalibratingActive;
extern volatile uint32_t   lastBleReportMs;
extern const uint32_t      BLE_REPORT_INTERVAL_MS;

extern volatile uint8_t    batteryLevel;
extern volatile unsigned long lastActivityTime; 
extern volatile int        sharedWhammyRaw;   
extern volatile int        sharedPickupRaw;
extern volatile uint32_t   sharedBatteryMv;
extern volatile int        sharedStrumUpRaw;
extern volatile int        sharedStrumDownRaw;

extern int                 strumUpZeroOffset;
extern int                 strumDownZeroOffset;
extern int                 strumUpMaxDelta;   
extern int                 strumDownMaxDelta; 
extern volatile bool       isHallEffectMode;
extern volatile bool       mechanicalFallback;
extern volatile uint8_t    currentPresetIndex;  
extern volatile uint32_t   presetShowStartMs;   
extern volatile bool       isCharging;
extern bool                isLeftHanded;
extern bool                accelInitialized;
extern bool                hapticInitialized;
extern int                 rawEma;
extern int16_t             lastSentWhammy;
extern int16_t             lastSentTilt;
extern bool                buttonsInitialized;
extern Bounce2::Button     buttons[BUTTON_COUNT];

// DRV2605 calibration data
extern uint8_t drv2605CalibrationData[3];