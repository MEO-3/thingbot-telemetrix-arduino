#pragma once

#define ARDUINO_ID 1

#define SPI_ENABLED 1
#define I2C_ENABLED 1
#define DHT_ENABLED 1
#define ULTRASONIC_ENABLED 1
#define THINGBOT_EXTENDED 1

// BLE transport settings — BLE_TRANSPORT / TRANSPORT_SWITCHABLE set via build flags
#define BLE_DEVICE_NAME   "ThingBot"

// Boot button transport switching (ESP32-C3 DevKitM-1)
#define BOOT_BUTTON_PIN   9      // BOOT button, active LOW
#define BOOT_HOLD_MS      2000   // hold duration to trigger transport switch
#define ONBOARD_LED_PIN   8      // built-in blue LED (active HIGH)
