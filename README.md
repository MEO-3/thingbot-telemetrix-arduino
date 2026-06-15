# ThingBot Telemetrix Arduino

ESP32-C3 firmware implementing a Telemetrix-style serial protocol for ThingBot hardware. A host (Python client) sends packetized commands over USB CDC serial; the firmware executes them and streams back sensor reports.

## Hardware

- **Board:** ESP32-C3 DevKitM-1
- **Serial:** USB CDC at 115200 baud
- **Extended hardware:** PCA9685 PWM driver (I2C) for DC motors, servos, buzzer, and LEDs

## Build

Requires [PlatformIO](https://platformio.org/).

```bash
# Build
pio run -e esp32-c3-devkitm-1

# Flash
pio run -e esp32-c3-devkitm-1 --target upload

# Monitor serial output
pio device monitor
```

See [`docs/build_release.md`](docs/build_release.md) for producing a single merged flashable binary.

## Protocol

Every packet starts with a **length byte** (count of bytes that follow), then a **command/report ID byte**, then arguments. Both directions use the same format.

| ID  | Name                | Direction          |
|-----|---------------------|--------------------|
| 0   | SERIAL_LOOP_BACK    | host ↔ device      |
| 1   | SET_PIN_MODE        | host → device      |
| 2   | DIGITAL_WRITE / DIGITAL_REPORT | both  |
| 4   | ANALOG_WRITE / ANALOG_REPORT   | both  |
| 6   | ARE_YOU_THERE / I_AM_HERE      | both  |
| 7   | READ_ULTRASONIC / ULTRASONIC_REPORT | both |
| 11  | DHT_REPORT          | device → host      |
| 101 | DC_WRITE            | host → device      |
| 102 | SERVO_WRITE / THINGBOT_SW_REPORT | host → device / device → host |
| 103 | BUZZER_WRITE        | host → device      |
| 104 | LED_WRITE           | host → device      |

## Feature Toggle

ThingBot extended hardware (PCA9685 motors, servos, LEDs, buzzer) is gated by a single flag in `lib/ThingBotTelemetrixArduino/core/config.h`:

```c
#define THINGBOT_EXTENDED 1   // set to 0 for base Telemetrix-compatible build
```

## Code Structure

```
src/
  main.cpp          — shared state, command table, dispatch, setup/loop
  handlers.cpp/h    — generic command handler implementations
  scan.cpp/h        — pin scan loops and timing state

lib/
  ThingBotTelemetrixArduino/
    core/
      config.h      — feature flags
      protocol.h    — command/report IDs and protocol constants
      pin_state.h   — pin/sensor structs and extern declarations
    ThingBotExtended.h/.cpp  — PCA9685 motor/servo/buzzer/LED layer
  Ultrasonic/       — minimal trigger/echo ultrasonic driver
```

See [`docs/specs/project_specs.md`](docs/specs/project_specs.md) for full protocol and feature details.

## License

AGPL-3.0 — see [`LICENSE`](LICENSE).
