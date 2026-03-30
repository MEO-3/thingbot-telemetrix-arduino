#pragma once
#include <Arduino.h>

class Ultrasonic {
public:
    Ultrasonic(uint8_t triggerPin, uint8_t echoPin);
    long readDistance();
    uint8_t getTriggerPin() {
        return triggerPin;
    }
    uint8_t getEchoPin() {
        return echoPin;
    }
private:
    uint8_t triggerPin;
    uint8_t echoPin;

};