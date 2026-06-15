#include "config.h"
#ifdef THINGBOT_EXTENDED

#include "ThingBotExtended.h"
#include "pin_state.h"
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

uint16_t map_speed_to_pwm(int value) {
    return (uint16_t)map(value, 0, 100, 0, 4095);
}

uint16_t map_angle_to_pwm(int angle) {
    return (uint16_t)map(angle, 0, 180, 150, 600);
}

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

#endif // THINGBOT_EXTENDED
