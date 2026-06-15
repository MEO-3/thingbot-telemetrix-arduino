#pragma once
#include "core/config.h"
#include "core/protocol.h"
#include "core/pin_state.h"
#include "core/transport.h"
#ifdef BLE_TRANSPORT
#include "transport/BLETransport.h"
#else
#include "transport/SerialTransport.h"
#endif
#ifdef THINGBOT_EXTENDED
#include "ThingBotExtended.h"
#endif
