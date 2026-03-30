#include"Ultrasonic.h"

Ultrasonic::Ultrasonic(uint8_t triggerPin, uint8_t echoPin) {
    this->triggerPin = triggerPin;
    this->echoPin = echoPin;
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

long Ultrasonic::readDistance() {
    // Send a 10 microsecond pulse to trigger the ultrasonic sensor
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);

    // Read the duration of the echo pulse
    long duration = pulseIn(echoPin, HIGH);

    // Calculate distance in centimeters (speed of sound is approximately 343 m/s)
    long distance = duration * 0.034 / 2;

    return distance;
}