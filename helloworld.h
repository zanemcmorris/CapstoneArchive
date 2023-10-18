#include "my_gpio.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

typedef enum{
  CO2,
  PRESSURE,
  PPM1,
  PPM25,
  PPM40,
  PPM10,
  HUMIDITY,
  TEMP,
  VOC,
  CO, 
  NG
}sensor_map;

#define LED_PIN_0 A3
#define LED_PIN_1 A1
