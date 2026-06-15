#pragma once
#include <Arduino.h>

// ThingBot Extended Command IDs
#define DC_WRITE     101
#define SERVO_WRITE  102
#define BUZZER_WRITE 103
#define LED_WRITE    104

// ThingBot Extended Report IDs
#define THINGBOT_SW_REPORT 102

// Motor identifiers
#define M1 1
#define M2 2
#define M3 3
#define M4 4

// Servo identifiers
#define S1 1
#define S2 2
#define S3 3
#define S4 4
#define S5 5

// PCA9685 channel assignments — DC motors
#define M1_A 2
#define M1_B 3
#define M2_A 4
#define M2_B 5
#define M3_A 7
#define M3_B 8
#define M4_A 1
#define M4_B 0

// PCA9685 channel assignments — Servos
#define SERVO_1 12
#define SERVO_2 11
#define SERVO_3 10
#define SERVO_4  9
#define SERVO_5  8

// PCA9685 channel assignments — Buzzer and LEDs
#define BUZZER 14
#define LED_1  15
#define LED_2  13

// ESP32-C3 GPIO board switch
#define SW 3

void     control_dc();
void     control_servo();
void     control_buzzer();
void     control_led();
uint16_t map_speed_to_pwm(int value);
uint16_t map_angle_to_pwm(int angle);
void     setup_pwm_driver();
void     setup_sw_input();
