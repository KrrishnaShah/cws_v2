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

typedef struct s_imu_data {
  uint32_t epoch_time;
  
  int acc_val_x;
  int acc_val_y;
  int acc_val_z;

  int gyro_val_x;
  int gyro_val_y;
  int gyro_val_z;
} s_imu_data_t;

typedef struct s_flash_header {
  uint32_t page;
  uint32_t addr;
  uint32_t count;
} s_flash_header_t;

PCF8563 rtc;
LSM6DS3 myIMU(I2C_MODE, IMU_I2C_ADDRESS);  // I2C device address 0x6A
SPIFlash flash(7);

static void __blinky(long blinky_ms);
static void __updateAccelerometer(void);



void setup() {
  pinMode(TEMP_POWER, OUTPUT);   // initialize the built-in LED pin to indicate when a central is connected
  pinMode(LED_BUILTIN, OUTPUT);  // initialize the built-in LED pin to indicate when a central is connected

  digitalWrite(TEMP_POWER, HIGH);

  Serial.begin(115200);

  if (myIMU.begin() != 0) {
    Serial.println("Accelerometer error");
  } else {
    Serial.println("Accelerometer OK!");
  }

  // if (!BLE.begin()) {
  //   Serial.println("starting BLE failed!");
  // }

  rtc.begin();
  if (rtc.isrunning()) {
    Serial.println("RTC is running.");
  } else {
    Serial.println("RTC is NOT running!");
    // rtc.adjust(DateTime(__DATE__, __TIME__));
    rtc.adjust(DateTime(2013, 5, 19, 11, 54, 00));
  }

  if (flash.begin(MB(16))) {
    Serial.println("Flash is running.");
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
  Serial.println(now.format(buf));
  uint32_t epoch32 = now.unixtime();
  Serial.print("epoch time now: ");
  Serial.println(epoch32);

  delay(1000);

  // LowPower.sleep(2000);
  // myPowSave.
}

static void __updateAccelerometer(void) {
  static int acc_val_x;
  static int acc_val_y;
  static int acc_val_z;

  static int gyro_val_x;
  static int gyro_val_y;
  static int gyro_val_z;

  acc_val_x = (int)(myIMU.readFloatAccelX() * 100);
  acc_val_y = (int)(myIMU.readFloatAccelY() * 100);
  acc_val_z = (int)(myIMU.readFloatAccelZ() * 100);

  gyro_val_x = (int)(myIMU.readFloatGyroX() * 100);
  gyro_val_y = (int)(myIMU.readFloatGyroY() * 100);
  gyro_val_z = (int)(myIMU.readFloatGyroZ() * 100);

  Serial.print("acc-x: ");
  Serial.print(acc_val_x / 100.0);
  Serial.print(", acc-y: ");
  Serial.print(acc_val_y / 100.0);
  Serial.print(", acc-z: ");
  Serial.print(acc_val_z / 100.0);

  Serial.print(", gyro-x: ");
  Serial.print(gyro_val_x / 100.0);
  Serial.print(", gyro-y: ");
  Serial.print(gyro_val_y / 100.0);
  Serial.print(", gyro-z: ");
  Serial.println(gyro_val_z / 100.0);
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
