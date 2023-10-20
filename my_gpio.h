#ifndef MY_GPIO_H
#define MY_GPIO_H

#include <Arduino.h>
#include "driver/i2c.h"
#include "driver/gpio.h"
#include <pas-co2-ino.hpp>
#include <Adafruit_Sensor.h>
#include <SensirionI2CSen5x.h>

#define I2C_FREQ_HZ 400000
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define PERIODIC_MEAS_INTERVAL_IN_SECONDS 10
#define PRESSURE_REFERENCE 900
// Standard GPIO Pin definitions
#define LED_OUT_3 A3
#define LED_OUT_2 A2
#define LED_OUT_1 A1
#define NGPin A0
#define COPin A1

  void setup_GPIO(void);
  void read_all_sensors(float *ret_array, uint16_t array_size);

#endif

