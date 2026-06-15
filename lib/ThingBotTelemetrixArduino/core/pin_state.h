#pragma once
#include <Arduino.h>
#include <DHT.h>
#include <Ultrasonic.h>
#include "protocol.h"

#define MAX_DIGITAL_PINS_SUPPORTED 32
#define MAX_ANALOG_PINS_SUPPORTED  32
#define AT_MODE_NOT_SET            0xFF

#define DHT_PIN_MODE       0x11
#define DHT_TYPE_11        11
#define DHT_TYPE_22        22
#define ULTRASONIC_PIN_MODE 0x12

struct pin_descriptor {
    byte pin_number;
    byte pin_mode;
    bool reporting_enabled;
    int  last_value;
    int  differential;
};

struct dht_sensor {
    DHT* dht_instance;
    byte dht_type;
};

struct ultrasonic_sensor {
    Ultrasonic* ultrasonic_instance;
};

extern byte              command_buffer[MAX_COMMAND_LENGTH];
extern pin_descriptor    the_digital_pins[MAX_DIGITAL_PINS_SUPPORTED];
extern pin_descriptor    the_analog_pins[MAX_ANALOG_PINS_SUPPORTED];
extern dht_sensor        dht_sensors[MAX_DIGITAL_PINS_SUPPORTED];
extern ultrasonic_sensor ultrasonic_sensors[MAX_DIGITAL_PINS_SUPPORTED];
