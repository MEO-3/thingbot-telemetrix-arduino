Project Specs - ThingBot Telemetrix Arduino

Overview
- Firmware implementing a Telemetrix-style serial protocol for ThingBot hardware.
- Target platform is ESP32-C3 (board: esp32-c3-devkitm-1) using the Arduino framework.
- Core loop parses serial commands, executes pin I/O and peripheral actions, and reports sensor changes.

Primary Goals
- Provide a lightweight command/response transport over Serial for pin control and sensor telemetry.
- Support common classroom/robotics peripherals: GPIO, analog I/O, DHT sensors, ultrasonic sensors, PWM-driven motors/servos, buzzer, LEDs, and a board switch.

Hardware/Platform
- MCU/Board: ESP32-C3 DevKitM-1.
- Serial: 115200 baud; packetized protocol with leading length byte.
- Libraries: DHT (Adafruit), Adafruit PWM Servo Driver (PCA9685), Adafruit Unified Sensor, and a local Ultrasonic driver.

Project Structure
- `src/main.cpp`: Core firmware, protocol handling, device scan loops, and extended ThingBot controls.
- `lib/Ultrasonic`: Minimal driver for trigger/echo ultrasonic sensors.
- `lib/ThingBotTelemetrixArduino`: Placeholder library stub (no implementation yet).
- `platformio.ini`: PlatformIO configuration, board, libs, and build flags.

Serial Protocol
- Packet format: first byte is packet length (number of bytes following), second byte is command ID, remaining bytes are arguments.
- Commands are dispatched via a command table indexed by command ID.
- Responses use the same leading-length format.

Command IDs
- 0: SERIAL_LOOP_BACK
- 1: SET_PIN_MODE
- 2: DIGITAL_WRITE
- 3: DIGITAL_READ
- 4: ANALOG_WRITE
- 5: ANALOG_READ
- 6: ARE_YOU_THERE
- 7: READ_ULTRASONIC
- 101: DC_WRITE (ThingBot extended)
- 102: SERVO_WRITE (ThingBot extended)
- 103: BUZZER_WRITE (ThingBot extended)
- 104: LED_WRITE (ThingBot extended)

Report IDs
- 2: DIGITAL_REPORT (same numeric value as DIGITAL_WRITE)
- 4: ANALOG_REPORT (same numeric value as ANALOG_WRITE)
- 6: I_AM_HERE (response to ARE_YOU_THERE)
- 7: ULTRASONIC_REPORT
- 11: DHT_REPORT
- 102: THINGBOT_SW_REPORT (board switch state change)

Supported Features
- GPIO
  - Set pin modes: INPUT, INPUT_PULLUP, OUTPUT, and custom DHT/Ultrasonic modes.
  - Digital reporting on state change when enabled per-pin.
- Analog I/O
  - Analog read with periodic sampling (default interval 19 ms).
  - Analog write via PWM (analogWrite).
- DHT Sensors
  - DHT11/DHT22 setup via SET_PIN_MODE.
  - Periodic read and report (default interval 3000 ms).
- Ultrasonic Sensors
  - Custom driver using trigger/echo; READ_ULTRASONIC scans configured sensors and reports distance in cm.
- ThingBot Extended (PCA9685)
  - DC motors M1-M4 with direction control.
  - Servos S1-S5 with angle to PWM mapping.
  - Buzzer frequency mapped to PWM.
  - LEDs with on/off control.
  - Board switch input (SW) reports state changes.

Runtime Flow
- `setup()` initializes Serial, pin structures, and PWM driver (if enabled).
- `loop()` continually:
  - Reads and dispatches incoming commands.
  - Scans digital inputs for changes.
  - Samples analog inputs and reports changed values.
  - Reads DHT sensors on interval.

Build/Upload
- PlatformIO environment: `env:esp32-c3-devkitm-1`.
- Build flags enable USB CDC for serial communication on boot.

Licensing
- AGPL-3.0 (see `LICENSE`).

Notable Gaps / TODOs
- `lib/ThingBotTelemetrixArduino` is a stub with no functionality.
- DIGITAL_READ and ANALOG_READ currently do not emit responses in the implementation (only read values locally).
