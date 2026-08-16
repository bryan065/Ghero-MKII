#pragma once
#include <Arduino.h>
#include <XboxGamepadDevice.h>
#include "ButtonTypes.h"

#define VERSION         "1.0.0"

// DISABLE DEBUG FLAG BEFORE FINAL PRODUCTION USE
#define DEBUG
//#define DEBUG_HALL

// ===================================================================
// Settings
// ===================================================================
#define NUM_LEDS            3
#define BATTERY_TIMEOUT     (10 * (60 * 1000))
#define HELPER_SLAP         XBOX_BUTTON_SELECT
#define BATTERY_CURVE       lg_hg2              // Battery Selection (generic, lg_hg2)
#define BUTTON_DEBOUNCE_MS  5
#define RAKE

// ===================================================================
// Hardware / ADC Pins
// ===================================================================
#define CHARGING_PIN    10
#define I2C_SDA_PIN     43
#define I2C_SCL_PIN     44
#define BATTERY_ADC_PIN 3
#define WHAMMY_ADC_PIN  9
#define PICKUP_ADC_PIN  4
#define TAP_INT_PIN     45
#define NEOPIXEL_PIN    5

#define STRUM_UP_PIN    6
#define STRUM_DOWN_PIN  7
#define WAKEUP_PIN      2

// ===================================================================
// Button Pin Mapping
// ===================================================================
constexpr size_t BUTTON_COUNT = 15;

// { GPIO_PIN, BUTTON_ID, DEBOUNCE_TIME }
const ButtonConfig BUTTON_MAP[BUTTON_COUNT] = {
  { 38,             HELPER_LT }, 
  { 39,             XBOX_BUTTON_LB },
  { 40,             XBOX_BUTTON_RB },
  { 41,             HELPER_RT }, 
  { 8,              XBOX_BUTTON_A },
  { 35,             XBOX_BUTTON_START },
  { 36,             XBOX_BUTTON_SELECT },
  { 1,              HELPER_FUNCTION }, 
  { 14,             HELPER_DPAD_UP }, 
  { 13,             HELPER_DPAD_DOWN }, 
  { 12,             HELPER_DPAD_LEFT }, 
  { 21,             HELPER_DPAD_RIGHT }, 
  { STRUM_UP_PIN,   HELPER_DPAD_UP }, 
  { STRUM_DOWN_PIN, HELPER_DPAD_DOWN }, 
  { WAKEUP_PIN,     XBOX_BUTTON_HOME }
};

// ===================================================================
// Pickguard Tap Sensitivity Settings (Lower value = Higher sensitivity / Less force required)
//    TAP_LOW, TAP_MED, TAP_HIGH, TAP_ULTRA
// ===================================================================
#define TAP_SENSITIVITY   TAP_MED
