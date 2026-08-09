#include <Arduino.h>
#include "ThingBotTelemetrixArduino.h"
#include "scan.h"

unsigned long current_millis;
unsigned long analog_previous_millis = 0;
unsigned long dht_previous_millis = 0;
uint8_t  analog_sampling_interval = 19;
uint16_t dht_read_interval = 3000; // milliseconds for accurate DHT readings

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
                transport->write(input_message, 4);
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

                    transport->write(input_message, 4);
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
                        transport->write(input_message, 5);
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

                // A failed read yields NAN, and casting NAN to int is undefined behaviour — the
                // first read after begin() always fails, so skip instead of reporting garbage.
                if (isnan(h) || isnan(t)) {
                    continue;
                }

                input_message[2] = (byte) i; // pin number

                // send humidity
                input_message[3] = highByte((int)(h * 100)); // send as integer * 100
                input_message[4] = lowByte((int)(h * 100));

                // send temperature
                input_message[5] = highByte((int)(t * 100)); // send as integer * 100
                input_message[6] = lowByte((int)(t * 100));
                transport->write(input_message, 7);
                // send_debug_info(DHT_REPORT, (int)(t * 100));
                // send_debug_info(DHT_REPORT, (int)(h * 100));
            }
        }
    }
}
