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

src/
- main.cpp: Shared state definitions, command table, dispatch (lookup_command, get_next_command), setup(), loop().
- handlers.h / handlers.cpp: All 8 generic command handler implementations (serial loopback, pin mode, digital/analog read/write, ultrasonic, are-you-there).
- scan.h / scan.cpp: Pin structure initialization and the three scan loops (digital, analog, DHT); owns all timing variables.

lib/ThingBotTelemetrixArduino/
- core/config.h: Feature flags (THINGBOT_EXTENDED, DHT_ENABLED, etc.) and ARDUINO_ID.
- core/protocol.h: Command/report ID definitions, DEBUG_REPORT, MAX_COMMAND_LENGTH, command_descriptor struct.
- core/pin_state.h: pin_descriptor, dht_sensor, ultrasonic_sensor structs; extern declarations for all shared state arrays.
- core/transport.h: Reserved for future transport-layer abstractions.
- ThingBotExtended.h: All ThingBot extended defines — motor/servo identifiers (M1-M4, S1-S5), PCA9685 channel assignments, extended command IDs (101-104), function declarations.
- ThingBotExtended.cpp: Implementations of control_dc, control_servo, control_buzzer, control_led, map helpers, setup_pwm_driver, setup_sw_input. Owns the Adafruit_PWMServoDriver instance.

lib/Ultrasonic/
- Minimal trigger/echo ultrasonic driver. Instances are created per-pin at runtime via SET_PIN_MODE.

platformio.ini: PlatformIO environment, board, lib_deps, and USB CDC build flags.

Serial Protocol
- Packet format: first byte is packet length (number of bytes following), second byte is command ID, remaining bytes are arguments.
- Commands are dispatched via a command table in main.cpp, indexed by command ID.
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
- 99: DEBUG_REPORT (debug utility, not part of normal protocol)
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
  - Custom driver using trigger/echo; READ_ULTRASONIC scans all configured sensors and reports distance.
- ThingBot Extended (PCA9685) — enabled via THINGBOT_EXTENDED in core/config.h
  - DC motors M1-M4 with direction control via paired PCA9685 channels.
  - Servos S1-S5 with angle-to-PWM mapping (0-180° → pulse 150-600).
  - Buzzer frequency mapped to PWM duty cycle.
  - LEDs (LED_1, LED_2) with on/off control.
  - Board switch input (SW, GPIO 3) reports state changes as THINGBOT_SW_REPORT.

Runtime Flow
- setup() initializes Serial, pin structures, and (if THINGBOT_EXTENDED) the PCA9685 driver and board switch.
- loop() continually:
  - Captures current_millis = millis() once per iteration.
  - Reads and dispatches one pending serial command.
  - Scans digital inputs for changed values and emits reports.
  - Samples analog inputs on a 19 ms interval and emits reports on change.
  - Reads DHT sensors on a 3000 ms interval and emits temperature/humidity reports.

Feature Toggle
- Set THINGBOT_EXTENDED to 0 (or comment it out) in core/config.h to build base Telemetrix-compatible firmware with no PCA9685 dependency.

Build/Upload
- PlatformIO environment: env:esp32-c3-devkitm-1.
- Build flags enable USB CDC for serial communication on boot.
- See docs/build_release.md for producing a merged flashable binary.

Licensing
- AGPL-3.0 (see LICENSE).

Notable Gaps / TODOs
- DIGITAL_READ and ANALOG_READ do not emit response packets (values are read locally but not reported).
- core/transport.h is reserved and currently empty.
