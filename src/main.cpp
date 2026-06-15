/*
  ThingBot Telemetrix Arduino Library
  Copyright (c) 2026 ThingEdu. All rights reserved.
  Based on Telemetrix4Arduino Library (Alan Yorinks)
*/

#include <Arduino.h>
#include "ThingBotTelemetrixArduino.h"

#ifdef I2C_ENABLED
#include <Wire.h>
#endif

#ifdef SPI_ENABLED
#include <SPI.h>
#endif

// Shared state — extern-declared in pin_state.h
byte             command_buffer[MAX_COMMAND_LENGTH];
pin_descriptor   the_digital_pins[MAX_DIGITAL_PINS_SUPPORTED];
pin_descriptor   the_analog_pins[MAX_ANALOG_PINS_SUPPORTED];
dht_sensor       dht_sensors[MAX_DIGITAL_PINS_SUPPORTED];
ultrasonic_sensor ultrasonic_sensors[MAX_DIGITAL_PINS_SUPPORTED];

// Forward declarations for generic command handlers
void serial_loopback();
void set_pin_mode();
void digital_write();
void digital_read();
void analog_write();
void analog_read();
void are_you_there();
void read_ultrasonic();

command_descriptor command_table[] = {
    {SERIAL_LOOP_BACK, &serial_loopback},
    {SET_PIN_MODE,     &set_pin_mode},
    {DIGITAL_WRITE,    &digital_write},
    {DIGITAL_READ,     &digital_read},
    {ANALOG_WRITE,     &analog_write},
    {ANALOG_READ,      &analog_read},
    {ARE_YOU_THERE,    &are_you_there},
    {READ_ULTRASONIC,  &read_ultrasonic},
#ifdef THINGBOT_EXTENDED
    {DC_WRITE,         &control_dc},
    {SERVO_WRITE,      &control_servo},
    {BUZZER_WRITE,     &control_buzzer},
    {LED_WRITE,        &control_led},
#endif
};

void (*lookup_command(byte command))() {
    size_t command_count = sizeof(command_table) / sizeof(command_table[0]);
    for (size_t i = 0; i < command_count; i++) {
        if (command_table[i].command_id == command) {
            return command_table[i].command_func;
        }
    }
    return nullptr;
}

void send_debug_info(byte id, int value) {
    byte debug_buffer[5] = {(byte)4, (byte)DEBUG_REPORT, 0, 0, 0 };
    debug_buffer[2] = id;
    debug_buffer[3] = highByte(value);
    debug_buffer[4] = lowByte(value);
    Serial.write(debug_buffer, 5);
}

void get_next_command() {
    byte command;
    byte packet_length;
    void (*command_func)();

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
    command_func = lookup_command(command);
    if (command_func == nullptr) {
        return;
    }

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
    command_func();
}

unsigned long current_millis;
unsigned long analog_previous_millis = 0;
unsigned long dht_previous_millis = 0;
uint8_t  analog_sampling_interval = 19;
uint16_t dht_read_interval = 3000; // milliseconds for accurate DHT readings

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
        case DHT_PIN_MODE:
            dht_sensors[pin].dht_type = command_buffer[2];
            if (dht_sensors[pin].dht_type == DHT_TYPE_11) {
                dht_sensors[pin].dht_instance = new DHT(pin, DHT11);
            } else if (dht_sensors[pin].dht_type == DHT_TYPE_22) {
                dht_sensors[pin].dht_instance = new DHT(pin, DHT22);
            }
            dht_sensors[pin].dht_instance->begin();
            the_digital_pins[pin].pin_mode = DHT_REPORT;
            break;
        case ULTRASONIC_PIN_MODE:
            // for ultrasonic sensors, command_buffer[2] = outPin (trigger pin)
            // create a new Ultrasonic instance and store it in the analog pin structure
            ultrasonic_sensors[pin].ultrasonic_instance = new Ultrasonic(command_buffer[2], pin);
            the_analog_pins[pin].pin_mode = ULTRASONIC_REPORT;
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

void read_ultrasonic() {
    for (int i = 0; i < MAX_ANALOG_PINS_SUPPORTED; i++) {
        if (the_analog_pins[i].pin_mode == ULTRASONIC_REPORT) {
            long distance = ultrasonic_sensors[i].ultrasonic_instance->readDistance();
            byte echo_pin = ultrasonic_sensors[i].ultrasonic_instance->getEchoPin();
            byte trigger_pin = ultrasonic_sensors[i].ultrasonic_instance->getTriggerPin();
            byte report_message[6] = {6, ULTRASONIC_REPORT, echo_pin, trigger_pin, highByte(distance), lowByte(distance)};
            Serial.write(report_message, 6);
            // send_debug_info(ULTRASONIC_REPORT, distance);
        }
    }
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

void scan_digital_inputs() {
    byte value;
    byte input_message[4] = {3, DIGITAL_REPORT, 0, 0};

    for (int i = 0; i < MAX_DIGITAL_PINS_SUPPORTED; i++) {
        #ifdef THINGBOT_EXTENDED
        if (i == SW) {
            // handle switch input separately
            value = (byte) digitalRead(SW);
            if (value != the_digital_pins[SW].last_value) {
                the_digital_pins[SW].last_value = value;
                input_message[1] = (byte) THINGBOT_SW_REPORT;
                input_message[2] = (byte) SW;
                input_message[3] = value;
                Serial.write(input_message, 4);
            }
            continue;
        }
        #endif

        if (the_digital_pins[i].pin_mode == INPUT || the_digital_pins[i].pin_mode == INPUT_PULLUP) {
            // send_debug_info(i, the_digital_pins[i].reporting_enabled);
            if (the_digital_pins[i].reporting_enabled) {
                // if the value changed since last read
                value = (byte) digitalRead(the_digital_pins[i].pin_number);
                // send_debug_info(i, value);
                if (value != the_digital_pins[i].last_value) {
                    the_digital_pins[i].last_value = value;
                    input_message[1] = DIGITAL_REPORT;
                    input_message[2] = (byte) i;
                    input_message[3] = value;
                    // send_debug_info(3, value);

                    Serial.write(input_message, 4);
                }
            }
        }
    }
}

void scan_analog_inputs() {
    int value;
    byte input_message[5] = {4, ANALOG_REPORT, 0, 0, 0};

    if (current_millis - analog_previous_millis > analog_sampling_interval) {
        analog_previous_millis += analog_sampling_interval;

        for (int i = 0; i < MAX_ANALOG_PINS_SUPPORTED; i++) {
            if (the_analog_pins[i].pin_mode == ANALOG) {
                if (the_analog_pins[i].reporting_enabled) {
                    // if the value changed since last read
                    value = analogRead(the_analog_pins[i].pin_number);

                    // send_debug_info(i, value);
                    if (value != the_analog_pins[i].last_value) {
                        // check to see if the trigger_threshold was achieved
                        // trigger_value = abs(value - the_analog_pins[i].last_value);

                        // if(trigger_value > the_analog_pins[i].trigger_threshold) {
                        // trigger value achieved, send out the report
                        the_analog_pins[i].last_value = value;
                        // input_message[1] = the_analog_pins[i].pin_number;
                        input_message[2] = (byte) i;
                        input_message[3] = highByte(value); // get high order byte
                        input_message[4] = lowByte(value);
                        Serial.write(input_message, 5);
                        delay(1);
                    }
                }
            }
        }
    }
}

void scan_dht_inputs() {
    float h;
    float t;
    byte input_message[7] = {6, DHT_REPORT, 0, 0, 0, 0, 0};

    if (current_millis - dht_previous_millis > dht_read_interval) {
        dht_previous_millis += dht_read_interval;
        for (int i = 0; i < MAX_DIGITAL_PINS_SUPPORTED; i++) {
            if (the_digital_pins[i].pin_mode == DHT_REPORT) {
                h = dht_sensors[i].dht_instance->readHumidity();
                t = dht_sensors[i].dht_instance->readTemperature();

                input_message[2] = (byte) i; // pin number

                // send humidity
                input_message[3] = highByte((int)(h * 100)); // send as integer * 100
                input_message[4] = lowByte((int)(h * 100));

                // send temperature
                input_message[5] = highByte((int)(t * 100)); // send as integer * 100
                input_message[6] = lowByte((int)(t * 100));
                Serial.write(input_message, 7);
                // send_debug_info(DHT_REPORT, (int)(t * 100));
                // send_debug_info(DHT_REPORT, (int)(h * 100));
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    init_pin_structures();
#ifdef THINGBOT_EXTENDED
    setup_pwm_driver();
    setup_sw_input();
#endif
}

void loop() {
    current_millis = millis();
    get_next_command();
    scan_digital_inputs();
    scan_analog_inputs();
    scan_dht_inputs();
}
