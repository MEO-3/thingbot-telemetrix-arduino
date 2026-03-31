Build Release Firmware

Overview
This project uses PlatformIO for builds. To create a single flashable image
(`full_firmware.bin`) that combines bootloader, partition table, boot_app0,
and application firmware, use `esptool.py merge_bin` from PlatformIO's tool
package.

Build
```bash
pio run -e esp32-c3-devkitm-1
```

Merge Binaries
```bash
pio pkg exec --package tool-esptoolpy -- esptool.py --chip esp32c3 merge_bin -o full_firmware.bin --flash_mode dio --flash_freq 80m --flash_size 4MB \
  0x0 .pio/build/esp32-c3-devkitm-1/bootloader.bin \
  0x8000 .pio/build/esp32-c3-devkitm-1/partitions.bin \
  0xe000 ~/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin \
  0x10000 .pio/build/esp32-c3-devkitm-1/firmware.bin
```

Notes
- `boot_app0.bin` is not generated in the build output; use the framework copy:
  `~/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin`.
- If the framework package is installed elsewhere, locate it with:
  `pio pkg show -p framework-arduinoespressif32`.
- The offsets must match the partition table. Adjust if using a custom layout.
