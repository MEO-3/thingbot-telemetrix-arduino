#pragma once
#include <Arduino.h>

class Transport {
public:
    virtual bool   connected()                              = 0;
    virtual int    available()                              = 0;
    virtual int    read()                                   = 0;
    virtual size_t write(const uint8_t* buf, size_t size)  = 0;
    virtual ~Transport() {}
};

extern Transport* transport;
