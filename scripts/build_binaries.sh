#!/bin/bash

echo "Build Serial Firmware version"
pio run -e esp32-c3-devkitm-1

pio pkg exec --package tool-esptoolpy -- esptool.py --chip esp32c3 merge_bin -o full_firmware.bin --flash_mode dio --flash_freq 80m --flash_size 4MB \
  0x0 .pio/build/esp32-c3-devkitm-1/bootloader.bin \
  0x8000 .pio/build/esp32-c3-devkitm-1/partitions.bin \
  0xe000 ~/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin \
  0x10000 .pio/build/esp32-c3-devkitm-1/firmware.bin

echo "Build BLE Firmware version"
pio run -e esp32-c3-devkitm-1-ble

pio pkg exec --package tool-esptoolpy -- esptool.py --chip esp32c3 merge_bin -o full_firmware_ble.bin --flash_mode dio --flash_freq 80m --flash_size 4MB \
  0x0 .pio/build/esp32-c3-devkitm-1-ble/bootloader.bin \
  0x8000 .pio/build/esp32-c3-devkitm-1-ble/partitions.bin \
  0xe000 ~/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin \
  0x10000 .pio/build/esp32-c3-devkitm-1-ble/firmware.bin

echo "Done"