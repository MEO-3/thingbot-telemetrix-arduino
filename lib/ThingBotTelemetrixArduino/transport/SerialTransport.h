#pragma once
#include <Arduino.h>
#include "core/transport.h"

template<typename SerialType>
class SerialTransport : public Transport {
public:
    SerialTransport(SerialType& serial, unsigned long baud)
        : _serial(serial), _baud(baud) {}

    void begin() {
        _serial.begin(_baud);
    }

    bool connected() override {
        return (bool)_serial;
    }

    int available() override {
        return _serial.available();
    }

    int read() override {
        return _serial.read();
    }

    size_t write(const uint8_t* buf, size_t size) override {
        return _serial.write(buf, size);
    }

private:
    SerialType&   _serial;
    unsigned long _baud;
};
