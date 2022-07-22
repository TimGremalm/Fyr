# Fyr
Lighthouse electronics for receiving DMX512.

# Specs
## Microcontroller
* MCU: ESP32 PICO D4 https://docs.espressif.com/projects/esp-idf/en/release-v3.0/get-started/get-started-pico-kit-v3.html
* PICO Pin Definitions https://demo-dijiudu.readthedocs.io/en/latest/get-started/get-started-pico-kit.html#pin-descriptions
* PICO D4 datasheet https://espressif.com/sites/default/files/documentation/esp32-pico-d4_datasheet_en.pdf
* ESP32 ADC https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/

## Electronics
* Light development board: LightboxNano https://github.com/TimGremalm/LightboxNano
* Hall effect sensor OH49E https://datasheetspdf.com/pdf-file/1418337/Ouzhuo/OH49E/1

# Inspiration
DMX parsing inspired by https://github.com/luksal/ESP32-DMX

# Setup
* Configure esp-idf environment (ESP-IDF v5.0-dev-4001-g495d35949d 2nd stage bootloader)
* https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html

## Rehash build environment
```bash
get-idf
```

# Build and flash

## Configure
```bash
idf.py menuconfig
```

## Build
```bash
idf.py build
```

## Flash
```bash
idf.py -p /dev/ttyUSB0 flash
```

# Serial Monitor
## Monitor
```bash
idf.py -p /dev/ttyUSB0 monitor
```

## Close monitor
Press <kbd>CTRL</kbd>+<kbd>]</kbd> to exit.

## Build and flash within monitor
Press <kbd>CTRL</kbd>+<kbd>T</kbd> then <kbd>A</kbd> to build and flash.

