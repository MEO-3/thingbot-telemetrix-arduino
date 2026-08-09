#pragma once

#define ARDUINO_ID 1

#define SPI_ENABLED 1
#define I2C_ENABLED 1
#define DHT_ENABLED 1
#define ULTRASONIC_ENABLED 1
#define THINGBOT_EXTENDED 1

// BLE transport settings — BLE_TRANSPORT / TRANSPORT_SWITCHABLE set via build flags
#define BLE_DEVICE_NAME   "ThingBot"

// GPIO 8 (SDA) and GPIO 9 (SCL) are the ESP32-C3 I2C bus: the PCA9685 at 0x40 and the OLED at
// 0x3c live there. Never drive either as a plain GPIO — doing so before Wire claims the pins
// leaves the bus stalled and every PCA9685 output (motors, servos, buzzer, LEDs) stays dead.
