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
#define DHT_ENABLED 1
#define THINGBOT_EXTENDED 1

#ifdef I2C_ENABLED
#include <Wire.h>
#endif

#ifdef SPI_ENABLED
#include <SPI.h>
#endif

#ifdef DHT_ENABLED
// DHT config
#define DHT_PIN_MODE 0x11
#define DHT_TYPE_11 11
#define DHT_TYPE_22 22
#endif

#ifdef THINGBOT_EXTENDED
#include <Adafruit_PWMServoDriver.h>

#define M1 1
#define M2 2
#define M3 3
#define M4 4

#define S1 1
#define S2 2
#define S3 3
#define S4 4
#define S5 5
#endif

// Command index (matches command_table index)
#define SERIAL_LOOP_BACK 0
#define SET_PIN_MODE 1
#define DIGITAL_WRITE 2
#define DIGITAL_READ 3
#define ANALOG_WRITE 4
#define ANALOG_READ 5
#define ARE_YOU_THERE 6

#define DC_WRITE 7
#define SERVO_WRITE 8
#define BUZZER_WRITE 9
#define LED_WRITE 10

extern void serial_loopback();

extern void set_pin_mode();

extern void digital_write();

extern void digital_read();

extern void analog_write();

extern void analog_read();

extern void are_you_there();

#ifdef THINGBOT_EXTENDED
extern void control_dc();

extern void control_servo();

extern void control_buzzer();

extern void control_led();

// PWM helper functions
extern uint16_t map_speed_to_pwm(int value);
extern uint16_t map_angle_to_pwm(int angle);
extern void setup_pwm_driver();
#endif

// Report types
#define DIGITAL_REPORT DIGITAL_WRITE
#define ANALOG_REPORT ANALOG_WRITE
#define I_AM_HERE 6
#define DHT_REPORT 11
#define THINGBOT_SW_REPORT 12

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
    &are_you_there,
    #ifdef THINGBOT_EXTENDED
    &control_dc,
    &control_servo,
    &control_buzzer,
    &control_led
    #endif
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
unsigned long analog_previous_millis = 0;  // for analog input loop
unsigned long dht_previous_millis = 0;  // for DHT read loop
uint8_t analog_sampling_interval = 19;
uint16_t dht_read_interval = 3000; // milliseconds for accurate DHT readings

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

// DHT sensor structure
struct dht_sensor {
    DHT* dht_instance;
    byte dht_type;
};
dht_sensor dht_sensors[MAX_DIGITAL_PINS_SUPPORTED];

// PCA9685 PWM Servo Driver
#ifdef THINGBOT_EXTENDED
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Pin definitions
#define M1_A 2
#define M1_B 3
#define M2_A 4
#define M2_B 5
#define M3_A 7
#define M3_B 8
#define M4_A 1
#define M4_B 0

#define SERVO_1 12
#define SERVO_2 11
#define SERVO_3 10
#define SERVO_4 9
#define SERVO_5 8

#define BUZZER 14
#define LED_1 15
#define LED_2 13

#define SW 3 // ESP32 C3 GPIO
#endif

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

#ifdef THINGBOT_EXTENDED
void setup_pwm_driver() {
    pwm.begin();
    pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
    delay(10);
}

void setup_sw_input() {
    pinMode(SW, INPUT_PULLUP);
    the_digital_pins[SW].pin_mode = INPUT_PULLUP;
    the_digital_pins[SW].reporting_enabled = true;
}

void control_dc() {
    byte motor;
    byte speed;
    motor = command_buffer[0];
    speed = command_buffer[1];
    // send_debug_info(DC_WRITE, speed);
    switch (motor) {
        case M1:
            if (speed >= 0) {
                pwm.setPWM(M1_A, 0, map_speed_to_pwm(speed));
                pwm.setPWM(M1_B, 0, 0);
            } else {
                pwm.setPWM(M1_A, 0, 0);
                pwm.setPWM(M1_B, 0, map_speed_to_pwm(-speed));
            }
            break;
        case M2:
            if (speed >= 0) {
                pwm.setPWM(M2_A, 0, map_speed_to_pwm(speed));
                pwm.setPWM(M2_B, 0, 0);
            } else {
                pwm.setPWM(M2_A, 0, 0);
                pwm.setPWM(M2_B, 0, map_speed_to_pwm(-speed));
            }
            break;
        case M3:
            if (speed >= 0) {
                pwm.setPWM(M3_A, 0, map_speed_to_pwm(speed));
                pwm.setPWM(M3_B, 0, 0);
            } else {
                pwm.setPWM(M3_A, 0, 0);
                pwm.setPWM(M3_B, 0, map_speed_to_pwm(-speed));
            }
            break;
        case M4:
            if (speed >= 0) {
                pwm.setPWM(M4_A, 0, map_speed_to_pwm(speed));
                pwm.setPWM(M4_B, 0, 0);
            } else {
                pwm.setPWM(M4_A, 0, 0);
                pwm.setPWM(M4_B, 0, map_speed_to_pwm(-speed));
            }
            break;
    }
}

void control_servo() {
    byte servo;
    byte angle;
    servo = command_buffer[0];
    angle = command_buffer[1];
    // send_debug_info(SERVO_WRITE, angle);
    switch (servo) {
        case S1:
            pwm.setPWM(SERVO_1, 0, map_angle_to_pwm(angle));
            break;
        case S2:
            pwm.setPWM(SERVO_2, 0, map_angle_to_pwm(angle));
            break;
        case S3:
            pwm.setPWM(SERVO_3, 0, map_angle_to_pwm(angle));
            break;
        case S4:
            pwm.setPWM(SERVO_4, 0, map_angle_to_pwm(angle));
            break;
        case S5:
            pwm.setPWM(SERVO_5, 0, map_angle_to_pwm(angle));
            break;
    }
}

void control_buzzer() {
    byte frequency;
    frequency = command_buffer[0];
    // send_debug_info(BUZZER_WRITE, frequency);
    if (frequency == 0) {
        pwm.setPWM(BUZZER, 0, 0);
    } else {
        pwm.setPWM(BUZZER, 0, map_speed_to_pwm(frequency));
    }
}

void control_led() {
    byte led;
    byte state;
    led = command_buffer[0];
    state = command_buffer[1];
    // send_debug_info(LED_WRITE, state);
    switch (led) {
        case 1:
            if (state) {
                pwm.setPWM(LED_1, 0, map_speed_to_pwm(100));
            } else {
                pwm.setPWM(LED_1, 0, 0);
            }
            break;
        case 2:
            if (state) {
                pwm.setPWM(LED_2, 0, map_speed_to_pwm(100));
            } else {
                pwm.setPWM(LED_2, 0, 0);
            }
            break;
    }
}

uint16_t map_speed_to_pwm(int value) {
    return (uint16_t)map(value, 0, 100, 0, 4095);
}

uint16_t map_angle_to_pwm(int angle) {
    return (uint16_t)map(angle, 0, 180, 150, 600);
}

#endif

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