#pragma once
#include <Arduino.h>

// Command IDs
#define SERIAL_LOOP_BACK 0
#define SET_PIN_MODE     1
#define DIGITAL_WRITE    2
#define DIGITAL_READ     3
#define ANALOG_WRITE     4
#define ANALOG_READ      5
#define ARE_YOU_THERE    6
#define READ_ULTRASONIC  7

// Report IDs
#define DIGITAL_REPORT    DIGITAL_WRITE
#define ANALOG_REPORT     ANALOG_WRITE
#define I_AM_HERE         6
#define ULTRASONIC_REPORT READ_ULTRASONIC
#define DHT_REPORT        11

#define DEBUG_REPORT       99
#define MAX_COMMAND_LENGTH 30

struct command_descriptor {
    byte command_id;
    void (*command_func)();
};
