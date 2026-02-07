/*
  ThingBot Telemetrix Arduino Library
  Copyright (c) 2026 ThingEdu. All rights reserved.
  Based on Telemetrix4Arduino Library (Alan Yorinks)
*/

#include <Arduino.h>
#include <DHT.h>

#define ARDUINO_ID 1

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

// Command index (matches command_table index)
#define SERIAL_LOOP_BACK 0
#define SET_PIN_MODE 1
#define DIGITAL_WRITE 2
#define DIGITAL_READ 3
#define ANALOG_WRITE 4
#define ANALOG_READ 5
#define ARE_YOU_THERE 6

#define I2C_WRITE 7
#define I2C_READ 8
#define SPI_WRITE 9
#define SPI_READ 10

#define DC_WRITE 11
#define DC_READ 12
#define SERVO_WRITE 13
#define SERVO_READ 14

#define BUZZER_WRITE 15
#define PWM_WRITE 16

extern void serial_loopback();

extern void set_pin_mode();

extern void digital_write();

extern void digital_read();

extern void analog_write();

extern void analog_read();

extern void are_you_there();

// Report types
#define DIGITAL_REPORT DIGITAL_WRITE
#define ANALOG_REPORT ANALOG_WRITE
#define I_AM_HERE 6

#define DEBUG_PRINT 99

#define MAX_COMMAND_LENGTH 30
struct command_descriptor {
    void (*command_func)();
};

command_descriptor command_table[] = {
    &serial_loopback,       // 0
    &set_pin_mode,          // 1...
    &digital_write,
    &digital_read,
    &analog_write,
    &analog_read,
    &are_you_there
};

byte command_buffer[MAX_COMMAND_LENGTH];

void send_debug_info(byte id, int value) {
    byte debug_buffer[5] = {(byte)4, (byte)DEBUG_PRINT, 0, 0, 0 };
    debug_buffer[2] = id;
    debug_buffer[3] = highByte(value);
    debug_buffer[4] = lowByte(value);
    Serial.write(debug_buffer, 5);
}

void get_next_command() {
    byte command;
    byte packet_length;
    command_descriptor command_entry;

    // clear the command buffer
    memset(command_buffer, 0, sizeof(command_buffer));

    // if there is no command waiting, then return
    if (!Serial.available()) {
        return;
    }
    // get the packet length
    packet_length = (byte)Serial.read();

    while (!Serial.available()) {
        delay(1);
    }

    // get the command byte
    command = (byte)Serial.read();

    // send_debug_info(packet_length, command);
    command_entry = command_table[command];

    if (packet_length > 1) {
        // get the data for that command
        for (int i = 0; i < packet_length - 1; i++) {
        // need this delay or data read is not correct
        while (not Serial.available()) {
            delay(1);
        }
        command_buffer[i] = (byte)Serial.read();
        // send_debug_info(i, command_buffer[i]);
        }
    }
    // call the command function
    command_entry.command_func();
}

#define MAX_DIGITAL_PINS_SUPPORTED 100
#define MAX_ANALOG_PINS_SUPPORTED 16

unsigned long current_millis;   // for analog input loop
unsigned long previous_millis;  // for analog input loop
uint8_t analog_sampling_interval = 19;

struct pin_descriptor {
    byte pin_number;
    byte pin_mode;
    bool reporting_enabled;  // If true, then send reports if an input pin
    int last_value;          // Last value read for input mode
    int differential;        // Differential value for analog pins
};
#define AT_MODE_NOT_SET 0xFF
pin_descriptor the_digital_pins[MAX_DIGITAL_PINS_SUPPORTED];
pin_descriptor the_analog_pins[MAX_ANALOG_PINS_SUPPORTED];

void serial_loopback() {
    byte loop_back_buffer[3] = {2, (byte)SERIAL_LOOP_BACK, command_buffer[0] };
    Serial.write(loop_back_buffer, 3);
}

void set_pin_mode() {
    byte pin;
    byte mode;
    pin = command_buffer[0];
    mode = command_buffer[1];

    switch (mode) {
        case INPUT:
            the_digital_pins[pin].pin_mode = mode;
            the_digital_pins[pin].reporting_enabled = command_buffer[2];
            pinMode(pin, INPUT);
            break;
        case INPUT_PULLUP:
            the_digital_pins[pin].pin_mode = mode;
            the_digital_pins[pin].reporting_enabled = command_buffer[2];
            pinMode(pin, INPUT_PULLUP);
            break;
        case OUTPUT:
            the_digital_pins[pin].pin_mode = mode;
            pinMode(pin, OUTPUT);
            break;
        default:
            break;
    }
}

void digital_write() {
    byte pin;
    byte value;
    pin = command_buffer[0];
    value = command_buffer[1];
    digitalWrite(pin, value);
    // send_debug_info(DIGITAL_REPORT, value);
}

void analog_write() {
    // command_buffer[0] = PIN, command_buffer[1] = value_msb,
    // command_buffer[2] = value_lsb
    byte pin;  // command_buffer[0]
    unsigned int value;

    pin = command_buffer[0];

    value = (command_buffer[1] << 8) + command_buffer[2];
    analogWrite(pin, value);
}

void digital_read() {
    byte pin;
    byte value;
    pin = command_buffer[0];
    value = digitalRead(pin);
    // send_debug_info(DIGITAL_REPORT, value);
}

void analog_read() {
    byte pin;
    int value;
    pin = command_buffer[0];
    value = analogRead(pin);
    // send_debug_info(ANALOG_REPORT, value);
}

void are_you_there() {
    // send_debug_info(I_AM_HERE, ARDUINO_ID);
    byte report_message[3] = {2, I_AM_HERE, ARDUINO_ID};
    Serial.write(report_message, 3);
}

// initialize the pin data structures
void init_pin_structures() {
  for (byte i = 0; i < MAX_DIGITAL_PINS_SUPPORTED; i++) {
    the_digital_pins[i].pin_number = i;
    the_digital_pins[i].pin_mode = AT_MODE_NOT_SET;
    the_digital_pins[i].reporting_enabled = false;
    the_digital_pins[i].last_value = 0;
  }

  // establish the analog pin array
  for (byte i = 0; i < MAX_ANALOG_PINS_SUPPORTED; i++) {
    the_analog_pins[i].pin_number = i;
    the_analog_pins[i].pin_mode = AT_MODE_NOT_SET;
    the_analog_pins[i].reporting_enabled = false;
    the_analog_pins[i].last_value = 0;
    the_analog_pins[i].differential = 0;
  }
}

void setup() {
    Serial.begin(115200);
    init_pin_structures();
}

void loop() {
    get_next_command();
}