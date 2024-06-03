# Bongo Cat Typing Helper

![ESP32 Box S3 3 with Bongo Cat](./images/keybo.png)

This program is the USBHostKeyboardBLE example with Bongo Cat animation.

This program works as a USB keyboard to BLE keyboard converter on an Espressif
ESP32-BOX-S3-3 with USB host dock. In addition, the cat performs on the screen
to keep you amused. See https://bongo.cat/ for more details.

I did not create Bongo Cat so this only for personal projects.

This program does not work as a USB mouse/joystick to BLE mouse/joystick
converter but the cat will play if a USB HID non-keyboard device is plugged in.

## ESP32 BLE Keyboard library

See https://github.com/T-vK/ESP32-BLE-Keyboard for installation instructions
and details.

See https://github.com/T-vK/ESP32-BLE-Keyboard#nimble-mode to patch the library
to use NimBLE. This sketch has only been tested using NimBLE.

## NimBLE library

Install "NimBLE-Arduino" by h2zero using the Arduino IDE Library Manager.

## PNG decode library

Install using the Arudino IDE Library Manager the library "PNGdec" by Larry Bank.

## Troubleshooting

If keystrokes do not appear on the BLE host, make sure NimBLE has been
enabled in ESP32-BLE-Keyboard.
