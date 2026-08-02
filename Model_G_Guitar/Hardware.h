#pragma once
#include <Arduino.h>
#include "Config.h"
#include "Globals.h"

void calibrateStrumZeroOnly();
void calibrateStrumFull();
void setAdxl345PowerState(bool enable);
void setupAdxl345Tap(uint8_t tapThreshold);
bool detectHallSensor(uint8_t pin);
void triggerStrumHaptic();
void playHapticEffect(uint8_t effect = 1);
void setHapticRTP(uint8_t intensity = 0);
void updateGamepadButton(uint16_t id, bool isPressed);
uint8_t getBatteryChargeLevel(uint32_t batteryMv, const BatteryProfile* curve, size_t numPoints);

template <size_t N>
inline uint8_t getBatteryChargeLevel(uint32_t batteryMv, const BatteryProfile (&curve)[N]) {
  return getBatteryChargeLevel(batteryMv, curve, N);
}