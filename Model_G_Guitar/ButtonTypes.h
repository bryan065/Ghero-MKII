// ButtonTypes.h
#pragma once
#include <Arduino.h>
#include <XboxGamepadDevice.h>

// Helper Xbox Macros & Button IDs
#define HELPER_LT             0x21
#define HELPER_RT             0x22
#define HELPER_DPAD_UP        0x23
#define HELPER_DPAD_DOWN      0x24
#define HELPER_DPAD_LEFT      0x25
#define HELPER_DPAD_RIGHT     0x26
#define HELPER_FUNCTION       0xFFFF
#define HALL_STRUM_UP         HELPER_DPAD_UP
#define HALL_STRUM_DOWN       HELPER_DPAD_DOWN

struct ButtonConfig {
  uint8_t  pin;
  uint16_t id;
  uint32_t debounceUs;
};