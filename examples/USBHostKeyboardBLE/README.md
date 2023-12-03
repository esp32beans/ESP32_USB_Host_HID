# USB Host Keyboard to BLE Keyboard

This example converts USB keyboard to BLE keyboard using an Espressif
ESP32-Box-S3-3 with USB host dock. This is the easiest way to use USB host on
ESP32-S3. The display it not used so will be blank.

## ESP32 BLE Keyboard library

See https://github.com/T-vK/ESP32-BLE-Keyboard for installation instructions
and details.

See https://github.com/T-vK/ESP32-BLE-Keyboard#nimble-mode to patch the library
to use NimBLE. This sketch has only been tested using NimBLE.

## NimBLE library

Install "NimBLE-Arduino" by h2zero using the Arduino IDE Library Manager.

## Troubleshooting

If keystrokes do not appear on the BLE host, make sure NimBLE has been
enabled in ESP32-BLE-Keyboard.
