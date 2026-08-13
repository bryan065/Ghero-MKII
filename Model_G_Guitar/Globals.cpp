#include "Globals.h"

float PRESET_SCALES[2][5] = {
  { 0.20f, 0.15f, 0.40f, 0.60f, 0.9f },
  { 0.20f, 0.15f, 0.40f, 0.60f, 0.9f }
};

const uint32_t PRESET_COLORS[5] = {
  0xFF0000, 0x00FF00, 0x0000FF, 0xFF00FF, 0xFFA500 
};

// Hardware Objects
Preferences prefs;
SemaphoreHandle_t xBleMutex = NULL;
SemaphoreHandle_t xI2cMutex = NULL;
SemaphoreHandle_t xTapSemaphore = NULL;
TaskHandle_t      rgbTaskHandle = NULL;
TaskHandle_t      hallEffectStrumTaskHandle = NULL;

XboxGamepadDevice* gamepad = nullptr;
BleCompositeHID compositeHID("Model G MKII™ Guitar", "G-SYSTEM", 100);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Adafruit_DRV2605 haptic;

#ifdef NEOPIXEL_PIN
  Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, NEOPIXEL_PIN, NEO_GRB);
#endif

// Global State Initializations
bool     lastDebouncedState[BUTTON_COUNT] = { false };
uint64_t lastStateChangeUs[BUTTON_COUNT] = { 0 };

volatile CalibState calState = CAL_NONE; 
volatile uint32_t   calPhaseStartMs = 0;   
volatile bool       isCalibratingActive = false;
volatile uint32_t   lastBleReportMs = 0;
const uint32_t      BLE_REPORT_INTERVAL_MS = 6;

volatile uint8_t    batteryLevel = 0;
volatile unsigned long lastActivityTime = 0; 
volatile int        sharedWhammyRaw = 0;   
volatile int        sharedPickupRaw = 0;
volatile uint32_t   sharedBatteryMv = 0;
volatile int        sharedStrumUpRaw = 0;
volatile int        sharedStrumDownRaw = 0;

int                 strumUpZeroOffset = 0;
int                 strumDownZeroOffset = 0;
int                 strumUpMaxDelta = 800;   
int                 strumDownMaxDelta = 800; 
volatile bool       isHallEffectMode = false;
volatile uint8_t    currentPresetIndex = 0;  
volatile uint32_t   presetShowStartMs = 0;   
volatile bool       isCharging = false;
bool                isLeftHanded = false;
bool                accelInitialized  = false;
bool                hapticInitialized = false;
int                 rawEma = -1;
int16_t             lastSentWhammy = -32768;
int16_t             lastSentTilt = -32768;

// DRV2605 calibration data
uint8_t             drv2605CalibrationData[3];