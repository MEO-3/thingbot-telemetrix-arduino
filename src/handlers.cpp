#include <Arduino.h>
#include "ThingBotTelemetrixArduino.h"

void serial_loopback() {
    byte loop_back_buffer[3] = {2, (byte)SERIAL_LOOP_BACK, command_buffer[0] };
    transport->write(loop_back_buffer, 3);
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
            transport->write(report_message, 6);
            // send_debug_info(ULTRASONIC_REPORT, distance);
        }
    }
}

void are_you_there() {
    // send_debug_info(I_AM_HERE, ARDUINO_ID);
    byte report_message[3] = {2, I_AM_HERE, ARDUINO_ID};
    transport->write(report_message, 3);
}
