https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json

Add:  || defined(ARDUINO_ARCH_NRF52840) in file SPIMemory.h to solve the issue <utils/delay.h>

run this command to fix the issue 'exec: "adafruit-nrfutil": executable file not found in $PATH'
  pip install adafruit-nrfutil