#pragma once
#include <Arduino.h>

void tapTask(void *pvParameters);
void adcTask(void *pvParameters);
void buttonTask(void *pvParameters);
void hallEffectStrumTask(void *pvParameters);
void rgbTask(void *pvParameters);
void tiltTask(void *pvParameters);
void whammyTask(void *pvParameters);
void batteryTask(void *pvParameters);
void IRAM_ATTR adxlTapISR();