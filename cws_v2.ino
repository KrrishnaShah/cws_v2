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
#include <UUID.h>
#include <RingBuf.h>

using namespace mbed;
using namespace rtos;
using namespace std::chrono_literals;

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


#define USE_FLASH 0

#define TEMP_POWER D3
#define IMU_I2C_ADDRESS 0x6A

#define BLE_MTU_SIZE (32 * 7)
#define DEVICE_NAME "Device-A"
#define DEVICE_NAME_LEN 16

const static char *device_uuid_str = "8f8a223c-04f2-11ee-be56-0242ac120002";
static uint8_t device_uuid[16] = { 0 };

#if (0 == USE_FLASH)
#define RING_BUFFER_SIZE 2048
#define RING_BUFFER_BLE_SEND 280

static uint32_t imu_data_count = 0;
RingBuf<s_imu_data_t, RING_BUFFER_SIZE> imu_ring_buffer;
#else
#define START_PAGE_NUMBER 2
#define FLASH_MAGIC_BYTES 0x12345678

typedef struct s_flash_header_v2 {
  uint32_t magic;
  uint32_t count;
  uint32_t write_addr;
  uint32_t read_addr;
} s_flash_header_v2_t;

static uint32_t flash_max_page = 0;
static uint32_t flash_max_capacity = 0;
static s_flash_header_v2_t flash_header_v2;

SPIFlash flash(7);

static int __spi_flash_write(uint32_t addr, uint8_t *data, uint32_t len);
static int __spi_flash_read(uint32_t addr, uint8_t *data, uint32_t len, uint32_t retry);
#endif

static const char *cws_data_char_uuid = "e202";
static const char *cws_time_char_uuid = "e203";
static const char *ble_gateway_name = "cws-gateway";

static s_imu_data_t imu_data;

RTC_PCF8563 rtc_pcf8563;
LSM6DS3 myIMU(I2C_MODE, IMU_I2C_ADDRESS);  // I2C device address 0x6A

static Thread ble_thread;
static Thread sensor_thread;

static Mutex spi_flash_mutex;

static void __blinky(long blinky_ms);
static void __updateAccelerometer(void);
static void __print_buffer(uint8_t *buffer, uint32_t len);

static void ble_thread_process(void);
static void sensor_thread_process(void);


void setup() {
  pinMode(TEMP_POWER, OUTPUT);   // initialize the built-in LED pin to indicate when a central is connected
  pinMode(LED_BUILTIN, OUTPUT);  // initialize the built-in LED pin to indicate when a central is connected

  digitalWrite(TEMP_POWER, HIGH);
  Serial.begin(250000);

  // while (!Serial) {
  //   delay(10);
  // }

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

  UUID uuid(device_uuid_str);
  uuid.setupLong(uuid.getBaseUUID(), UUID::MSB);
  uint8_t *base_uuid = (uint8_t *)uuid.getBaseUUID();
  memcpy(device_uuid, base_uuid, 16);

  rtc_pcf8563.adjust(DateTime(2023, 1, 1, 0, 0, 0));

#if (1 == USE_FLASH)
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
#endif

  sensor_thread.start(sensor_thread_process);
  ble_thread.start(ble_thread_process);
}

static void ble_thread_process(void) {
  Serial.println("BLE thread is running");
  ThisThread::sleep_for(5s);
  for (;;) {
#if (1 == USE_FLASH)
    if (flash_header_v2.read_addr + (BLE_MTU_SIZE * 5) < flash_header_v2.write_addr) {
      DateTime now = rtc_pcf8563.now();
      imu_data.epoch_time = now.unixtime();
      Serial.print("epoch before ble: ");
      Serial.println(imu_data.epoch_time);

      send_to_ble_from_flash();

      now = rtc_pcf8563.now();
      imu_data.epoch_time = now.unixtime();
      Serial.print("epoch after ble: ");
      Serial.println(imu_data.epoch_time);
    } else {
      ThisThread::sleep_for(100ms);
    }
#else
    if (imu_ring_buffer.size() > RING_BUFFER_BLE_SEND) {
      Serial.print(__LINE__);
      Serial.print(": BLE-thread:: Ring buffer read_pointer: ");
      Serial.println(imu_ring_buffer.size());

      uint32_t retries = 5;
      while (!send_to_ble_from_ring_buffer() && (retries--)) {
        ThisThread::sleep_for(20ms);
      }
    }

    ThisThread::sleep_for(500);
#endif
  }
}

static void sensor_thread_process(void) {
  Serial.println("Sensor thread is running");
  ThisThread::sleep_for(5s);
  for (;;) {
    __updateAccelerometer();
#if (1 == USE_FLASH)
    __write_imu_data_to_flash_v2();
#else
    __ring_buffer_write();
    __print_buffer((uint8_t *)&imu_data, sizeof(s_imu_data_t));
    Serial.println("");
    // ThisThread::sleep_for(85ms);
    ThisThread::sleep_for(85ms);
#endif
  }
}

void loop() {
  digitalWrite(LED_BUILTIN, 1);
  delay(2000);
}

static void __updateAccelerometer(void) {
  Serial.print(imu_data_count);
  imu_data.acc_val_x = (int)(myIMU.readFloatAccelX() * 100);
  imu_data.acc_val_y = (int)(myIMU.readFloatAccelY() * 100);
  imu_data.acc_val_z = (int)(myIMU.readFloatAccelZ() * 100);

  imu_data.gyro_val_x = (int)(myIMU.readFloatGyroX() * 100);
  imu_data.gyro_val_y = (int)(myIMU.readFloatGyroY() * 100);
  imu_data.gyro_val_z = (int)(myIMU.readFloatGyroZ() * 100);

  imu_data.sn = imu_data_count++;

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
#if (1 == USE_FLASH)
  Serial.print(flash_header_v2.count);
#else
#endif
  Serial.print(": ");
  Serial.println(imu_data.epoch_time);
  if (now.year() < 2023) {
#if (1 == USE_FLASH)
    send_to_ble_from_flash();
#endif
  }
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

    if (break_val)
      break;
  }
  Serial.println("");
}

#if (0 == USE_FLASH)
static int send_to_ble_from_ring_buffer(void) {
  int ret = 0;
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
        uint8_t ble_buffer[BLE_MTU_SIZE + DEVICE_NAME_LEN] = { 0 };
        snprintf((char *)ble_buffer, DEVICE_NAME_LEN, "%s", DEVICE_NAME);

        while (imu_ring_buffer.size() >= RING_BUFFER_BLE_SEND) {
          bool write_success = false;
          uint32_t item_read = __ring_buffer_read(ble_buffer + DEVICE_NAME_LEN, 7);
          if (item_read) {
            Serial.print(__LINE__);
            Serial.println("Sending to ble: ");
            __print_buffer(ble_buffer, (item_read * sizeof(s_imu_data_t)) + DEVICE_NAME_LEN);

            uint32_t retries = 5;
            while(retries-- && (false == write_success)) {
              Serial.print(__LINE__);
              Serial.print(": BLE: data write faield - ");
              Serial.println(retries);
              write_success = imu_data_char.writeValue(ble_buffer, (item_read * sizeof(s_imu_data_t) + DEVICE_NAME_LEN));
            }

            if (!peripheral.connected()) {
              break;
            }
          } else {
            Serial.println("374 - __ring_buffer_read failed!");
          }

          ThisThread::sleep_for(1ms);

          if (write_success) {
            ret = 1;
          }
        }

        time_t epoch_time = 0;
        if (imu_time_char.readValue(&epoch_time, sizeof(time_t))) {
          rtc_pcf8563.adjust(DateTime(epoch_time));
        } else {
          Serial.println("393 - Ble time read failed!");
        }

        BLE.disconnect();
        Serial.println(epoch_time);
      } else {
        Serial.println("399 - BLE connection failed!");
      }
    } else {
      Serial.println("402 - Failed to connect!");
    }
  } else {
    Serial.println("BLE server not found!");
    BLE.scanForName(ble_gateway_name);
  }

  return ret;
}


static void __ring_buffer_write(void) {
  if (imu_ring_buffer.isFull()) {
    s_imu_data_t tmp_imu_data;
    imu_ring_buffer.pop(tmp_imu_data);
    Serial.println("Ring buffer is full");
  }

  if (imu_ring_buffer.push(imu_data)) {
    Serial.println("data wrote to ring-buffer");
  }

  Serial.print(__LINE__);
  Serial.print(": Data count in ringbuffer: ");
  Serial.println(imu_ring_buffer.size());
}

static uint32_t __ring_buffer_read(uint8_t *data, uint32_t count) {
  uint32_t ret = 0;
  s_imu_data_t tmp_imu_data;
  for (int idx = 0; idx < count; idx++) {
    if (imu_ring_buffer.pop(tmp_imu_data)) {
      ret += 1;
    } else {
      break;
    }
    memcpy(data + (idx * sizeof(s_imu_data_t)), &tmp_imu_data, sizeof(s_imu_data_t));
  }

  return ret;
}
#endif

#if (1 == USE_FLASH)
static void send_to_ble_from_flash(void) {
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

        // sensor_thread_hold = 1;

        while ((flash_header_v2.read_addr + BLE_MTU_SIZE) < flash_header_v2.write_addr) {
          uint32_t byte_len_diff = (flash_header_v2.write_addr - flash_header_v2.read_addr);
          uint32_t read_len = byte_len_diff > BLE_MTU_SIZE ? BLE_MTU_SIZE : byte_len_diff;

          Serial.print("BLE-Read Address: ");
          Serial.print(flash_header_v2.read_addr);
          Serial.print(", diff: ");
          Serial.println(byte_len_diff);
          Serial.print(", real_len: ");
          Serial.println(read_len);

          // if (flash.readByteArray(flash_header_v2.read_addr, ble_buffer, read_len)) {
          if (__spi_flash_read(flash_header_v2.read_addr, ble_buffer, read_len, 2)) {
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
          ThisThread::sleep_for(5ms);
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

static int __spi_flash_read(uint32_t addr, uint8_t *data, uint32_t len, uint32_t retry) {
  int ret = 0;
  if (data && len) {
    while (retry--) {
      if (spi_flash_mutex.trylock_for(10ms)) {
        Serial.println("Read lock");
        if (flash.readByteArray(addr, data, len)) {
          ret = 1;
        }
        spi_flash_mutex.unlock();
        Serial.println("Read unlock");
        if (ret) {
          break;
        }
        ThisThread::sleep_for(2ms);
      } else {
        Serial.println("Read lock failed!");
      }
    }
  }
  return ret;
}

static int __spi_flash_write(uint32_t addr, uint8_t *data, uint32_t len) {
  int ret = 0;
  if (data && len) {
    Serial.println("Write lock");
    spi_flash_mutex.lock();
    if (flash.writeByteArray(addr, data, len)) {
      ret = 1;
    }
    spi_flash_mutex.unlock();
    Serial.println("Write unlock");
  }
  return ret;
}
#endif


#if (1 == USE_FLASH)
static int __flash_write() {
  int ret = 0;
  Serial.print("Flash Write address: ");
  Serial.println(flash_header_v2.write_addr);
  __print_buffer((uint8_t *)&imu_data, sizeof(s_imu_data_t));

  // if (flash.writeByteArray(flash_header_v2.write_addr, (uint8_t *)&imu_data, sizeof(s_imu_data_t))) {
  if (__spi_flash_write(flash_header_v2.write_addr, (uint8_t *)&imu_data, sizeof(s_imu_data_t))) {
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

  // if (!flash.readByteArray(flash_header_v2.write_addr - sizeof(s_imu_data_t), tmp_data, 32)) {
  if (!__spi_flash_read(flash_header_v2.write_addr - sizeof(s_imu_data_t), tmp_data, sizeof(tmp_data), 1)) {
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
    while (1) {
      Serial.print(".");
      ThisThread::sleep_for(1s);
    }
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
  // if (!flash.writeByteArray(0, (uint8_t *)&flash_header_v2, sizeof(s_flash_header_v2_t))) {
  if (!__spi_flash_write(0, (uint8_t *)&flash_header_v2, sizeof(s_flash_header_v2_t))) {
    if (flash.eraseSector(0)) {
      __spi_flash_write(0, (uint8_t *)&flash_header_v2, sizeof(s_flash_header_v2_t));
      // flash.writeByteArray(0, (uint8_t *)&flash_header_v2, sizeof(s_flash_header_v2_t));
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
#endif

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
