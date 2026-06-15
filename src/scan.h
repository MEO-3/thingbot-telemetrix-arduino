#pragma once
#include <Arduino.h>

extern unsigned long current_millis;

void init_pin_structures();
void scan_digital_inputs();
void scan_analog_inputs();
void scan_dht_inputs();
