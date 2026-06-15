#include "transport/BLETransport.h"

#ifdef BLE_TRANSPORT

BLETransport::BLETransport(const char *deviceName)
    : _deviceName(deviceName),
      _server(nullptr),
      _txChar(nullptr),
      _connected(false),
      _rxHead(0),
      _rxTail(0),
      _mux(portMUX_INITIALIZER_UNLOCKED) {}

void BLETransport::begin()
{
    Serial.begin(115200);
    Serial.println("[BLE] init start");

    NimBLEDevice::init(_deviceName);
    Serial.println("[BLE] device init done");

    _server = NimBLEDevice::createServer();
    _server->setCallbacks(this);

    NimBLEService *service = _server->createService(SERVICE_UUID);

    NimBLECharacteristic *rxChar = service->createCharacteristic(
        RX_CHAR_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    rxChar->setCallbacks(this);

    _txChar = service->createCharacteristic(
        TX_CHAR_UUID,
        NIMBLE_PROPERTY::NOTIFY);

    service->start();
    Serial.println("[BLE] service started");

    NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->start();
    Serial.println("[BLE] advertising started");
}

// --- Transport interface ---

bool BLETransport::connected()
{
    return _connected;
}

int BLETransport::available()
{
    return (int)_count();
}

int BLETransport::read()
{
    if (_count() == 0)
        return -1;
    return (int)_pop();
}

size_t BLETransport::write(const uint8_t *buf, size_t size)
{
    if (!_connected || _txChar == nullptr)
        return 0;
    size_t sent = 0;
    while (sent < size)
    {
        size_t chunk = min((size_t)BLE_MTU_SAFE, size - sent);
        _txChar->setValue(buf + sent, chunk);
        _txChar->notify();
        sent += chunk;
    }
    return sent;
}

// --- NimBLE server callbacks ---

void BLETransport::onConnect(NimBLEServer *server)
{
    _connected = true;
}

void BLETransport::onDisconnect(NimBLEServer *server)
{
    _connected = false;
    // restart advertising so a new host can connect
    NimBLEDevice::getAdvertising()->start();
}

// --- NimBLE characteristic callback ---

void BLETransport::onWrite(NimBLECharacteristic *characteristic)
{
    std::string data = characteristic->getValue();
    for (size_t i = 0; i < data.length(); i++)
    {
        _push((uint8_t)data[i]);
    }
}

// --- Ring buffer ---

void BLETransport::_push(uint8_t byte)
{
    portENTER_CRITICAL(&_mux);
    uint16_t next = (_rxHead + 1) % BLE_RX_BUF_SIZE;
    if (next != _rxTail)
    { // drop silently on overflow
        _rxBuf[_rxHead] = byte;
        _rxHead = next;
    }
    portEXIT_CRITICAL(&_mux);
}

uint8_t BLETransport::_pop()
{
    portENTER_CRITICAL(&_mux);
    uint8_t byte = _rxBuf[_rxTail];
    _rxTail = (_rxTail + 1) % BLE_RX_BUF_SIZE;
    portEXIT_CRITICAL(&_mux);
    return byte;
}

uint16_t BLETransport::_count()
{
    portENTER_CRITICAL(&_mux);
    uint16_t count = (_rxHead - _rxTail + BLE_RX_BUF_SIZE) % BLE_RX_BUF_SIZE;
    portEXIT_CRITICAL(&_mux);
    return count;
}

#endif // BLE_TRANSPORT
