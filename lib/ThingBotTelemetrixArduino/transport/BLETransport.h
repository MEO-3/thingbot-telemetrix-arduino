#pragma once
#ifdef BLE_TRANSPORT

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <string>
#include "core/transport.h"

#define SERVICE_UUID "aa700001-8f6a-4e2c-b369-4060e0bb33aa"
#define RX_CHAR_UUID "aa700002-8f6a-4e2c-b369-4060e0bb33aa"
#define TX_CHAR_UUID "aa700003-8f6a-4e2c-b369-4060e0bb33aa"

#define BLE_RX_BUF_SIZE 256
#define BLE_MTU_SAFE 20 // conservative; sufficient for all current report sizes

class BLETransport : public Transport,
                     public NimBLEServerCallbacks,
                     public NimBLECharacteristicCallbacks
{
public:
    BLETransport(const char *deviceName);
    void begin();

    // Transport interface
    bool connected() override;
    int available() override;
    int read() override;
    size_t write(const uint8_t *buf, size_t size) override;

    // NimBLE server callbacks
    void onConnect(NimBLEServer *server) override;
    void onDisconnect(NimBLEServer *server) override;

    // NimBLE characteristic callback — fires when host writes to RX characteristic
    void onWrite(NimBLECharacteristic *characteristic) override;

private:
    const char *_deviceName;
    std::string _fullName;
    NimBLEServer *_server;
    NimBLECharacteristic *_txChar;
    volatile bool _connected;

    // Ring buffer (single-producer BLE task, single-consumer loop task)
    uint8_t _rxBuf[BLE_RX_BUF_SIZE];
    volatile uint16_t _rxHead;
    volatile uint16_t _rxTail;
    portMUX_TYPE _mux;

    void _push(uint8_t byte);
    uint8_t _pop();
    uint16_t _count();
};

#endif // BLE_TRANSPORT
