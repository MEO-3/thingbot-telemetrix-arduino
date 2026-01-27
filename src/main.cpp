/*
  ThingBot Telemetrix Arduino Library
  Copyright (c) 2026 ThingEdu. All rights reserved.
  Based on Telemetrix4Arduino Library (Alan Yorinks)
*/

#include <Arduino.h>

#define SPI_ENABLED 1
#define I2C_ENABLED 1
#define THINGBOT_EXTENDED 1

#ifdef I2C_ENABLED
#include <Wire.h>
#endif

#ifdef SPI_ENABLED
#include <SPI.h>
#endif

#ifdef THINGBOT_EXTENDED
#include <Adafruit_PWMServoDriver.h>
#endif



void setup() {

}

void loop() {

}