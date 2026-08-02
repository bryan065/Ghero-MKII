#pragma once
#include <Arduino.h>

void tapTaskCore1(void *pvParameters);
void adcTaskCore1(void *pvParameters);
void buttonTaskCore1(void *pvParameters);
void hallEffectStrumTaskCore1(void *pvParameters);
void rgbTaskCore0(void *pvParameters);
void tiltTaskCore0(void *pvParameters);
void whammyTaskCore0(void *pvParameters);
void batteryTaskCore0(void *pvParameters);
void IRAM_ATTR adxlTapISR();