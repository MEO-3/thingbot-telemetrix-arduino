# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

```bash
# Build firmware
pio run -e esp32-c3-devkitm-1

# Upload to connected ESP32-C3
pio run -e esp32-c3-devkitm-1 --target upload

# Monitor serial output at 115200 baud
pio device monitor

# Build and create a single merged flashable image
pio run -e esp32-c3-devkitm-1
pio pkg exec --package tool-esptoolpy -- esptool.py --chip esp32c3 merge_bin -o full_firmware.bin --flash_mode dio --flash_freq 80m --flash_size 4MB \
  0x0 .pio/build/esp32-c3-devkitm-1/bootloader.bin \
  0x8000 .pio/build/esp32-c3-devkitm-1/partitions.bin \
  0xe000 ~/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin \
  0x10000 .pio/build/esp32-c3-devkitm-1/firmware.bin
```

`boot_app0.bin` is not in the build output — use the framework copy at `~/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin`. If installed elsewhere, find it with `pio pkg show -p framework-arduinoespressif32`.

## Architecture

This is Arduino/PlatformIO firmware for an ESP32-C3 that implements a Telemetrix-style serial command/response protocol. A host machine (Python client) sends packetized commands over USB CDC serial; the firmware executes them and streams back sensor reports.

**Serial Protocol:** Every packet starts with a length byte (count of bytes that follow), then a command/report ID byte, then arguments. Both host→device commands and device→host reports use this same format.

**Command dispatch (`src/main.cpp`):** `get_next_command()` reads the length and command ID, looks up the handler in `command_table[]` via `lookup_command()`, reads the remaining bytes into `command_buffer[]`, then calls the handler. All command handlers read their arguments from `command_buffer`.

**Main loop scanning:** `loop()` runs four functions every iteration:
1. `get_next_command()` — process one pending serial command
2. `scan_digital_inputs()` — report changed digital pin values
3. `scan_analog_inputs()` — report changed analog values every 19 ms
4. `scan_dht_inputs()` — report DHT temperature/humidity every 3000 ms

**Feature flags** (defined at top of `src/main.cpp`): `THINGBOT_EXTENDED` gates PCA9685 PWM driver code for DC motors (M1–M4), servos (S1–S5), buzzer, and LEDs. Disabling it produces a base Telemetrix-compatible build.

**PCA9685 pin mapping (THINGBOT_EXTENDED):** Motors use paired A/B PWM channels; direction is controlled by setting one channel to a PWM value and the other to 0. Servos map angle (0–180°) to PCA9685 pulse counts (150–600). `map_speed_to_pwm()` maps 0–100% to 0–4095.

**Local libraries:**
- `lib/Ultrasonic/` — minimal trigger/echo driver; instances are stored per-pin in `ultrasonic_sensors[]` and polled by `read_ultrasonic()` on command or `READ_ULTRASONIC` request.
- `lib/ThingBotTelemetrixArduino/` — stub class, not yet implemented.

## Command and Report ID Reference

| ID  | Name               | Direction     |
|-----|--------------------|---------------|
| 0   | SERIAL_LOOP_BACK   | host→device, device→host |
| 1   | SET_PIN_MODE       | host→device   |
| 2   | DIGITAL_WRITE / DIGITAL_REPORT | both |
| 4   | ANALOG_WRITE / ANALOG_REPORT   | both |
| 6   | ARE_YOU_THERE / I_AM_HERE      | both |
| 7   | READ_ULTRASONIC / ULTRASONIC_REPORT | both |
| 11  | DHT_REPORT         | device→host   |
| 101 | DC_WRITE           | host→device   |
| 102 | SERVO_WRITE / THINGBOT_SW_REPORT | host→device / device→host |
| 103 | BUZZER_WRITE       | host→device   |
| 104 | LED_WRITE          | host→device   |

## Known Gaps

- `DIGITAL_READ` (cmd 3) and `ANALOG_READ` (cmd 5) read values but do not emit a response packet.
- `lib/ThingBotTelemetrixArduino` is an empty stub.
