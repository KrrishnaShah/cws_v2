/*
  Temperature Monitor

  This example creates a Bluetooth® Low Energy peripheral with the standard temperature service and
  level characteristic. The A0 pin is used to calculate the temperature.

  The circuit:
  - Arduino MKR WiFi 1010, Arduino Uno WiFi Rev2 board, Arduino Nano 33 IoT,
    Arduino Nano 33 BLE, or Arduino Nano 33 BLE Sense board.

  You can use a generic Bluetooth® Low Energy central app, like LightBlue (iOS and Android) or
  nRF Connect (Android), to interact with the services and characteristics
  created in this sketch.

  This example code is in the public domain.
*/

#include <Wire.h>
#include <LSM6DS3.h>
#include <Arduino.h>
#include <ArduinoBLE.h>
// #include <ArduinoLowPower.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_I2CDevice.h>
#include <RTClib.h>
#include <SPIMemory.h>

#define DEVICE_ID 4
#define TEMP_POWER D3
#define IMU_I2C_ADDRESS 0x6A

#define START_PAGE_NUMBER 2
#define FLASH_MAGIC_BYTES 0x12345678
#define FLASH_PAGE_SIZE_BYTES 256


typedef struct s_imu_data {
  uint32_t epoch_time;

  int acc_val_x;
  int acc_val_y;
  int acc_val_z;

  int gyro_val_x;
  int gyro_val_y;
  int gyro_val_z;
} s_imu_data_t;

// typedef struct

typedef struct s_flash_header {
  uint32_t magic;
  uint32_t page;
  uint32_t addr;
} s_flash_header_t;

static s_flash_header_t flash_header;

RTC_PCF8563 rtc;
LSM6DS3 myIMU(I2C_MODE, IMU_I2C_ADDRESS);  // I2C device address 0x6A
SPIFlash flash(7);

static void __blinky(long blinky_ms);
static void __updateAccelerometer(void);

void setup() {
  pinMode(TEMP_POWER, OUTPUT);   // initialize the built-in LED pin to indicate when a central is connected
  pinMode(LED_BUILTIN, OUTPUT);  // initialize the built-in LED pin to indicate when a central is connected

  digitalWrite(TEMP_POWER, HIGH);
  Serial.begin(115200);

  while (!Serial) { delay(10); }


  if (myIMU.begin() != 0) {
    Serial.println("Accelerometer error");
  } else {
    Serial.println("Accelerometer OK!");
  }

  rtc.begin();
  if (rtc.isrunning()) {
    Serial.println("RTC is running.");
  } else {
    Serial.println("RTC is NOT running!");
    // rtc.adjust(DateTime(__DATE__, __TIME__));
  }

  rtc.adjust(DateTime(2013, 12, 31, 23, 59, 45));

  if (flash.begin()) {
    flash_header.addr = 0;
    flash_header.magic = 0;
    flash_header.page = 2;

    Serial.println("Flash is running.");
    Serial.print("flash.getCapacity: ");
    Serial.println(flash.getCapacity());

    Serial.print("flash.getMaxPage: ");
    Serial.println(flash.getMaxPage());

    if (FLASH_MAGIC_BYTES != flash_header.magic) {
      flash.readByteArray(0, (uint8_t *)&flash_header, sizeof(s_flash_header_t), 0);
    }
    Serial.print("flash-header-magic: ");
    Serial.println(flash_header.magic);
    Serial.print("flash-header-page: ");
    Serial.println(flash_header.page);
    Serial.print("flash-header-addr: ");
    Serial.println(flash_header.addr);
  } else {
    Serial.println("Flash is not running!");
  }
}

void loop() {
  digitalWrite(LED_BUILTIN, 1);
  // __updateAccelerometer();

  DateTime now = rtc.now();
  char buf[100];
  strncpy(buf, "DD.MM.YYYY hh:mm:ss", 100);
  Serial.println(now.toString(buf));

  uint32_t epoch32 = now.unixtime();
  Serial.print("epoch time now: ");
  Serial.println(epoch32);

  DateTime converted_time(epoch32);
  strncpy(buf, "converted -> DD.MM.YYYY hh:mm:ss", 100);
  Serial.println(converted_time.toString(buf));

  delay(1000);

  // LowPower.sleep(2000);
  // myPowSave.
}

static void __updateAccelerometer(void) {
  s_imu_data_t imu_data;
  memset(&imu_data, 0, sizeof(s_imu_data_t));

  imu_data.acc_val_x = (int)(myIMU.readFloatAccelX() * 100);
  imu_data.acc_val_y = (int)(myIMU.readFloatAccelY() * 100);
  imu_data.acc_val_z = (int)(myIMU.readFloatAccelZ() * 100);

  imu_data.gyro_val_x = (int)(myIMU.readFloatGyroX() * 100);
  imu_data.gyro_val_y = (int)(myIMU.readFloatGyroY() * 100);
  imu_data.gyro_val_z = (int)(myIMU.readFloatGyroZ() * 100);

  Serial.print("acc-x: ");
  Serial.print(imu_data.acc_val_x / 100.0);
  Serial.print(", acc-y: ");
  Serial.print(imu_data.acc_val_y / 100.0);
  Serial.print(", acc-z: ");
  Serial.print(imu_data.acc_val_z / 100.0);

  Serial.print(", gyro-x: ");
  Serial.print(imu_data.gyro_val_x / 100.0);
  Serial.print(", gyro-y: ");
  Serial.print(imu_data.gyro_val_y / 100.0);
  Serial.print(", gyro-z: ");
  Serial.println(imu_data.gyro_val_z / 100.0);

  flash.writeByteArray(0, 0, 0);
  // flash.read

  __update_flash_header();
}

static int __write_data_to_flash(s_imu_data_t *__imu_data) {
  if (__imu_data) {
    uint8_t imu_buffer[256];
    if (flash.readByteArray(flash_header.page, imu_buffer, 256)) {
      memcpy(imu_buffer + flash_header.addr, (uint8_t *)__imu_data, sizeof(s_imu_data_t));
      flash.writeByteArray(flash_header.page, imu_buffer, 256);
    }
  }
}

static int __update_flash_header(void) {
  // flash_header.addr

  if (flash_header.addr + sizeof(s_imu_data_t) > FLASH_PAGE_SIZE_BYTES) {
    if (flash_header.page < flash.getMaxPage()) {
      flash_header.page += 1;
      flash_header.addr = 0;
    } else {
      Serial.println("Warnnig: Memory is full!");
    }
  } else {
    flash_header.addr += sizeof(s_imu_data_t);
  }
}

static void __blinky(long blinky_ms) {
  static int _on;
  static long last_blink_ms;

  if (_on) {
    long current_ms = millis();
    if ((current_ms - last_blink_ms) > 100) {
      digitalWrite(LED_BUILTIN, 1);
      last_blink_ms = current_ms;
      _on = 0;
    }
  } else {
    long current_ms = millis();
    if ((current_ms - last_blink_ms) > blinky_ms) {
      digitalWrite(LED_BUILTIN, 0);
      last_blink_ms = current_ms;
      _on = 1;
    }
  }
}
