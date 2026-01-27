#ifndef THINGBOTTELEMETRIXARDUINO_H
#define THINGBOTTELEMETRIXARDUINO_H

#define COMMAND_BUFFER_SIZE 30

// Command define
#define SERIAL_LOOP_BACK 0x00
#define DIGITAL_WRITE 0x01
#define DIGITAL_READ 0x02
#define ANALOG_WRITE 0x03
#define ANALOG_READ 0x04
#define SET_PIN_MODE 0x05

#define I2C_WRITE 0x06
#define I2C_READ 0x07
#define SPI_WRITE 0x08
#define SPI_READ 0x09

#define DC_WRITE 0x0A
#define DC_READ 0x0B
#define SERVO_WRITE 0x0C
#define SERVO_READ 0x0D

#define BUZZER_WRITE 0x0E
#define PWM_WRITE 0x0F


class ThingBotTeletrixArduino {

public:
};

#endif // THINGBOTTELEMETRIXARDUINO_H