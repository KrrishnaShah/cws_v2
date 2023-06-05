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
#include <mbed.h>
#include <Wire.h>

using namespace mbed;
using namespace rtos;
using namespace std::chrono_literals;

#define DEVICE_ID 4
#define TEMP_POWER D3
#define IMU_I2C_ADDRESS 0x6A

#define START_PAGE_NUMBER 2
#define FLASH_MAGIC_BYTES 0x12345678
#define BLE_MTU_SIZE (32 * 7)

#define DEVICE_ID


typedef struct s_imu_data {
  uint32_t sn;
  uint32_t epoch_time;

  int acc_val_x;
  int acc_val_y;
  int acc_val_z;

  int gyro_val_x;
  int gyro_val_y;
  int gyro_val_z;
} s_imu_data_t;

typedef struct s_flash_header_v2 {
  uint32_t magic;
  uint32_t count;
  uint32_t write_addr;
  uint32_t read_addr;
} s_flash_header_v2_t;


static const char *ble_gateway_name = "cws-gateway";
static const char *cws_data_char_uuid = "e202";
// static const char *cws_data_char_uuid = "0000e202-0000-1000-8000-00805f9b34fb";
static const char *cws_time_char_uuid = "e203";
// static const char *cws_time_char_uuid = "0000e203-0000-1000-8000-00805f9b34fb";

SPIFlash flash(7);
RTC_PCF8563 rtc_pcf8563;
LSM6DS3 myIMU(I2C_MODE, IMU_I2C_ADDRESS);  // I2C device address 0x6A

static uint32_t flash_max_page = 0;
static uint32_t flash_max_capacity = 0;
static s_flash_header_v2_t flash_header_v2;
static s_imu_data_t imu_data;

static Thread ble_thread;
static Thread sensor_thread;

static volatile bool sensor_thread_hold = 0;

static void __blinky(long blinky_ms);
static s_imu_data_t *__updateAccelerometer(void);
static void __print_buffer(uint8_t *buffer, uint32_t len);
static void ble_thread_process(void);
static void sensor_thread_process(void);

static int __flash_write() {
  int ret = 0;
  Serial.print("Flash Write address: ");
  Serial.println(flash_header_v2.write_addr);
  __print_buffer((uint8_t *)&imu_data, sizeof(s_imu_data_t));

  if (flash.writeByteArray(flash_header_v2.write_addr, (uint8_t *)&imu_data, sizeof(s_imu_data_t))) {
    Serial.println("Flash write success");
    ret = 1;
    flash_header_v2.write_addr += sizeof(s_imu_data_t);
    if (flash_header_v2.write_addr > flash_max_capacity) {
      flash_header_v2.write_addr = 4096;
      Serial.println("Flash is full!");
    }
  } else {
    Serial.println("Flash write failed!");
  }

#if 1
  uint8_t tmp_data[32];
  Serial.print("Flash Read address: ");
  Serial.println(flash_header_v2.write_addr - sizeof(s_imu_data_t));

  if (!flash.readByteArray(flash_header_v2.write_addr - sizeof(s_imu_data_t), tmp_data, 32)) {
    Serial.println("Read failed!");
  } else {
    // flash_header_v2.read_addr += sizeof(s_imu_data_t);
  }

  if (0 == memcmp(&imu_data, tmp_data, 32)) {
    Serial.println("Checked: Wrote successfully");
  } else {
    Serial.println("**** Error: Data did not match!");

    Serial.print("Written data: len - ");
    Serial.println(sizeof(s_imu_data_t));
    __print_buffer((uint8_t *)&imu_data, sizeof(s_imu_data_t));

    Serial.println("Wrote data: len - 32");
    __print_buffer(tmp_data, 32);
    ThisThread::sleep_for(30s);
  }
#endif

  return ret;
}

static int __write_imu_data_to_flash_v2(void) {
#if 0
  Serial.println("writing data: ");
  Serial.print("Address: ");
  Serial.print(flash_header_v2.write_addr);
  Serial.print(", Size: ");
  Serial.println(sizeof(s_imu_data_t));
  __print_buffer((uint8_t *)&imu_data, sizeof(s_imu_data_t));
#endif
  imu_data.sn = flash_header_v2.count;

  if (!__flash_write()) {
    Serial.println("Flash write failed once!");
    if (!__flash_write()) {
      Serial.println("Flash write failed twice!");
      if (flash.eraseSector(flash_header_v2.write_addr)) {
        if (!__flash_write()) {
          Serial.println("Flash write failed thrice!");
        }
      }
    }
  }

  flash_header_v2.count += 1;

  if (!flash.writeByteArray(0, (uint8_t *)&flash_header_v2, sizeof(s_flash_header_v2_t))) {
    if (flash.eraseSector(0)) {
      flash.writeByteArray(0, (uint8_t *)&flash_header_v2, sizeof(s_flash_header_v2_t));
    }
  }
}

static int __flash_header_v2_init(void) {
  int ret = 0;
  memset(&flash_header_v2, 0, sizeof(s_flash_header_v2_t));
  flash.readByteArray(0, (uint8_t *)&flash_header_v2, sizeof(s_flash_header_v2_t));

  if (FLASH_MAGIC_BYTES != flash_header_v2.magic) {
    flash_header_v2.magic = FLASH_MAGIC_BYTES;
    flash_header_v2.count = 0;
    flash_header_v2.read_addr = 4096;
    flash_header_v2.write_addr = 4096;
  } else {
    Serial.println("Flash magic (V2) match found");
  }

  Serial.print("Flash header (V2) count: ");
  Serial.println(flash_header_v2.count);
  Serial.print("Flash header (V2) write_addr: ");
  Serial.println(flash_header_v2.write_addr);
  Serial.print("Flash header (V2) read_addr: ");
  Serial.println(flash_header_v2.read_addr);

  return ret;
}

void setup() {
  pinMode(TEMP_POWER, OUTPUT);   // initialize the built-in LED pin to indicate when a central is connected
  pinMode(LED_BUILTIN, OUTPUT);  // initialize the built-in LED pin to indicate when a central is connected

  digitalWrite(TEMP_POWER, HIGH);
  Serial.begin(250000);

  while (!Serial) { delay(10); }

  if (myIMU.begin() != 0) {
    Serial.println("Accelerometer error");
  } else {
    // Serial.println("Accelerometer OK!");
  }

  rtc_pcf8563.begin();
  if (rtc_pcf8563.isrunning()) {
    // Serial.println("RTC is running.");
  } else {
    Serial.println("RTC is NOT running!");
    // rtc_pcf8563.adjust(DateTime(__DATE__, __TIME__));
  }

  rtc_pcf8563.adjust(DateTime(2023, 1, 1, 0, 0, 0));

  if (flash.begin()) {
    Serial.println("Flash is running.");

    flash.eraseChip();
    flash.eraseSector(0);

    Serial.print("flash.getCapacity: ");
    flash_max_capacity = flash.getCapacity();
    Serial.println(flash_max_capacity);

    flash_max_page = flash.getMaxPage();
    Serial.print("flash.getMaxPage: ");
    Serial.println(flash_max_page);

    __flash_header_v2_init();

  } else {
    Serial.println("Flash is not running!");
  }

  sensor_thread.start(sensor_thread_process);
  ble_thread.start(ble_thread_process);
}

static void send_to_ble(void) {
  BLE.begin();
  BLE.scanForName(ble_gateway_name);

  BLEDevice peripheral = BLE.available();
  if (peripheral) {
    BLE.stopScan();

    if (peripheral.connect()) {
      if (!peripheral.discoverAttributes()) {
        Serial.println("Attribute discovery failed!");
        peripheral.disconnect();
      }

      BLECharacteristic imu_data_char = peripheral.characteristic(cws_data_char_uuid);
      if (!imu_data_char) {
        Serial.println("Peripheral does not have imu characteristic!");
        peripheral.disconnect();
      } else if (!imu_data_char.canWrite()) {
        Serial.println("Peripheral does not have a writable imu characteristic!");
        peripheral.disconnect();
      }

      BLECharacteristic imu_time_char = peripheral.characteristic(cws_time_char_uuid);
      if (!imu_time_char) {
        Serial.println("Peripheral does not have imu characteristic!");
        peripheral.disconnect();
      } else if (!imu_time_char.canRead()) {
        Serial.println("Peripheral does not have a readable imu characteristic!");
        peripheral.disconnect();
      }

      if (peripheral.connected()) {

        uint8_t ble_buffer[BLE_MTU_SIZE];

        sensor_thread_hold = 1;

        while ((flash_header_v2.read_addr + (BLE_MTU_SIZE * 10)) < flash_header_v2.write_addr) {
          Serial.print("Read Address: ");
          Serial.println(flash_header_v2.read_addr);

          uint32_t byte_len_diff = (flash_header_v2.write_addr - flash_header_v2.read_addr);
          uint32_t read_len = sizeof(s_imu_data_t); //  byte_len_diff > BLE_MTU_SIZE ? BLE_MTU_SIZE : byte_len_diff;
          Serial.print("real_len: ");
          Serial.println(read_len);

          if (flash.readByteArray(flash_header_v2.read_addr, ble_buffer, read_len)) {
            Serial.println("Flash read buffer: ");
            __print_buffer(ble_buffer, read_len);
            if (imu_data_char.writeValue(ble_buffer, read_len)) {
              flash_header_v2.read_addr += read_len;
            } else {
              Serial.println("BLE write Failed");
              if (!peripheral.connected()) {
                break;
              }
            }
          }
        }


        sensor_thread_hold = 0;

        time_t epoch_time = 0;
        if (imu_time_char.readValue(&epoch_time, sizeof(time_t))) {
          rtc_pcf8563.adjust(DateTime(epoch_time));
        } else {
          Serial.println("Ble time read failed!");
        }

        BLE.disconnect();
        Serial.println(epoch_time);

      } else {
        Serial.println("BLE connection failed!");
      }
    } else {
      Serial.println("Failed to connect!");
    }
  }
}

static void ble_thread_process(void) {
  Serial.println("BLE thread is running");
  ThisThread::sleep_for(5s);
  for (;;) {
    if (flash_header_v2.read_addr + (32 * 7 * 10) < flash_header_v2.write_addr) {
      DateTime now = rtc_pcf8563.now();
      imu_data.epoch_time = now.unixtime();
      Serial.print("epoch before ble: ");
      Serial.println(imu_data.epoch_time);

      send_to_ble();

      now = rtc_pcf8563.now();
      imu_data.epoch_time = now.unixtime();
      Serial.print("epoch after ble: ");
      Serial.println(imu_data.epoch_time);
    } else {
      ThisThread::sleep_for(100ms);
    }
  }
}

static void sensor_thread_process(void) {
  Serial.println("Sensor thread is running");
  ThisThread::sleep_for(5s);
  for (;;) {
    s_imu_data_t *tmp_imu_data = __updateAccelerometer();
    __write_imu_data_to_flash_v2();
    if (sensor_thread_hold) {
      ThisThread::sleep_for(10s);
    }
    // ThisThread::sleep_for(20ms);
  }
}

void loop() {
  digitalWrite(LED_BUILTIN, 1);
  delay(2000);
}

static s_imu_data_t *__updateAccelerometer(void) {
  imu_data.acc_val_x = (int)(myIMU.readFloatAccelX() * 100);
  imu_data.acc_val_y = (int)(myIMU.readFloatAccelY() * 100);
  imu_data.acc_val_z = (int)(myIMU.readFloatAccelZ() * 100);

  imu_data.gyro_val_x = (int)(myIMU.readFloatGyroX() * 100);
  imu_data.gyro_val_y = (int)(myIMU.readFloatGyroY() * 100);
  imu_data.gyro_val_z = (int)(myIMU.readFloatGyroZ() * 100);

#if 0
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
#endif

  DateTime now = rtc_pcf8563.now();
  imu_data.epoch_time = now.unixtime();
  Serial.print(flash_header_v2.count);
  Serial.print(": ");
  Serial.println(imu_data.epoch_time);
  if (now.year() < 2023) {
    send_to_ble();
  }

  return &imu_data;
}

static void __print_buffer(uint8_t *data_buffer, uint32_t len) {
  bool break_val = 0;
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 16; j++) {
      if (i * 16 + j < len) {
        char prnt_buffer[8];
        uint8_t x = data_buffer[i * 16 + j];
        // if (x < 10) Serial.print("0");
        // if (x < 100) Serial.print("0");

        if (j < 15) {
          snprintf(prnt_buffer, sizeof(prnt_buffer), "%02X, ", x);
          Serial.print(prnt_buffer);
        } else {
          snprintf(prnt_buffer, sizeof(prnt_buffer), "%02X", x);
          Serial.println(prnt_buffer);
        }
      } else {
        break_val = 1;
        break;
      }
    }

    if (break_val) break;
  }
  Serial.println("");
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
