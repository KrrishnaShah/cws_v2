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
#define FLASH_PAGE_SIZE_BYTES sizeof(imu_sector_buffer)


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

typedef struct s_flash_header {
  uint32_t magic;
  uint32_t sector;
  uint32_t page;
  uint32_t addr;
  uint32_t count;
} s_flash_header_t;

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

RTC_PCF8563 rtc_pcf8563;
LSM6DS3 myIMU(I2C_MODE, IMU_I2C_ADDRESS);  // I2C device address 0x6A
SPIFlash flash(7);
static s_flash_header_t flash_header;
static uint32_t flash_max_page = 0;
static uint32_t flash_max_capacity = 0;
static s_flash_header_v2_t flash_header_v2;
static s_imu_data_t imu_data;

static void __blinky(long blinky_ms);
static s_imu_data_t *__updateAccelerometer(void);
static void __print_buffer(uint8_t *buffer, uint32_t len);

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

  if (flash.writeByteArray(flash_header_v2.write_addr, (uint8_t *)&imu_data, sizeof(s_imu_data_t))) {
#if 0
    Serial.println("Flash write success!");
#endif
    flash_header_v2.write_addr += sizeof(s_imu_data_t);
    if (flash_header_v2.write_addr > flash_max_capacity) {
      flash_header_v2.write_addr = 4096;
      Serial.println("Flash is full!");
    }
  } else {
    Serial.println("Flash write failed once!");
    if (flash.eraseSector(flash_header_v2.write_addr)) {
      if (flash.writeByteArray(flash_header_v2.write_addr, (uint8_t *)&imu_data, sizeof(s_imu_data_t))) {
        flash_header_v2.write_addr += sizeof(s_imu_data_t);
      }
    }
  }

#if 0
  uint8_t tmp_buffer[256] = { 0 };
  if (flash.readByteArray(flash_header_v2.write_addr - sizeof(s_imu_data_t), (uint8_t *)tmp_buffer, sizeof(s_imu_data_t))) {
    Serial.println("Reading wrote data: ");
    __print_buffer(tmp_buffer, sizeof(s_imu_data_t));
  } else {
    Serial.println("Flash read failed!");
  }
#endif

  flash_header_v2.count += 1;

  if (!flash.writeByteArray(0, (uint8_t *)&flash_header_v2, sizeof(s_flash_header_v2_t))) {
    if (flash.eraseSector(0)) {
      flash.writeByteArray(0, (uint8_t *)&flash_header_v2, sizeof(s_flash_header_v2_t));
    }
  }
}

static int __flash_header_v2_init(void) {
  int ret = 0;
  if (flash.readByteArray(0, (uint8_t *)&flash_header_v2, sizeof(s_flash_header_v2_t))) {
    Serial.println("Flash magic (V2) match found");
    Serial.print("Flash header (V2) count: ");
    Serial.println(flash_header_v2.count);
    Serial.print("Flash header (V2) write_addr: ");
    Serial.println(flash_header_v2.write_addr);
    Serial.print("Flash header (V2) read_addr: ");
    Serial.println(flash_header_v2.read_addr);
  }

  if (FLASH_MAGIC_BYTES != flash_header_v2.magic) {
    flash_header_v2.magic = FLASH_MAGIC_BYTES;
    flash_header_v2.count = 0;
    flash_header_v2.read_addr = 4096;
    flash_header_v2.write_addr = 4096;
  }

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

    Serial.print("flash.getCapacity: ");
    flash_max_capacity = flash.getCapacity();
    Serial.println(flash_max_capacity);

    flash_max_page = flash.getMaxPage();
    Serial.print("flash.getMaxPage: ");
    Serial.println(flash_max_page);

    __flash_header_v2_init();

#if 0
    flash.readByteArray(0, (uint8_t *)&flash_header, sizeof(s_flash_header_t), 0);
    if (FLASH_MAGIC_BYTES != flash_header.magic) {
      flash_header.addr = 0;
      flash_header.magic = 0;
      flash_header.page = 16;   // each page size 256 Bytes
      flash_header.sector = 1;  // each sector size 4096 Bytes
      flash_header.count = 0;
      // flash.eraseChip();
    }

    Serial.print("flash-header-magic: ");
    Serial.println(flash_header.magic);
    Serial.print("flash-header-page: ");
    Serial.println(flash_header.page);
    Serial.print("flash-header-addr: ");
    Serial.println(flash_header.addr);
#endif
  } else {
    Serial.println("Flash is not running!");
  }
}


static void send_to_ble(void) {
  BLE.begin();
  BLE.scanForName(ble_gateway_name);

  BLEDevice peripheral = BLE.available();
  if (peripheral) {
    BLE.stopScan();

    if (peripheral.connect()) {
    } else {
      Serial.println("Failed to connect!");
    }

    if (peripheral.discoverAttributes()) {
    } else {
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
      uint8_t ble_buffer[512];

      while (flash_header_v2.read_addr <= flash_header_v2.write_addr) {
        uint32_t byte_len_diff = (flash_header_v2.write_addr - flash_header_v2.read_addr);
        uint32_t read_len = byte_len_diff > 512 ? 512 : byte_len_diff;
        if (flash.readByteArray(flash_header_v2.read_addr, ble_buffer, read_len)) {
          if (imu_data_char.writeValue(ble_buffer, read_len)) {
            flash_header_v2.read_addr += 512;
          } else {
            Serial.println("BLE write Failed");
          }
        }
      }

      time_t epoch_time = 0;
      if (imu_time_char.readValue(&epoch_time, sizeof(time_t))) {
        rtc_pcf8563.adjust(DateTime(epoch_time));
      } else {
        Serial.println("Ble time read failed!");
      }

      BLE.disconnect();
      Serial.println(epoch_time);
      rtc_pcf8563 now;

    } else {
      Serial.println("BLE connection failed!");
    }
  }
}

void loop() {
  digitalWrite(LED_BUILTIN, 1);
  s_imu_data_t *tmp_imu_data = __updateAccelerometer();
  __write_imu_data_to_flash_v2();

  if (flash_header_v2.read_addr + (16 * 32 * 10) < flash_header_v2.write_addr) {
    send_to_ble();

    DateTime now = rtc_pcf8563.now();
    imu_data.epoch_time = now.unixtime();
    Serial.print("epoch now: ");
    Serial.println(imu_data.epoch_time);
  }

#if 0
  BLEDevice peripheral = BLE.available();
  if (peripheral) {
#if 0
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();
#endif

    BLE.stopScan();
    // Serial.println("Connecting ...");

    if (peripheral.connect()) {
      // Serial.println("Connected");
    } else {
      Serial.println("Failed to connect!");
    }

    // discover peripheral attributes
    // Serial.println("Discovering attributes ...");
    if (peripheral.discoverAttributes()) {
      // Serial.println("Attributes discovered");
    } else {
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

    while (peripheral.connected()) {
      s_imu_data_t *tmp_imu_data = __updateAccelerometer();
      imu_data_char.writeValue(tmp_imu_data, sizeof(s_imu_data_t));
      time_t epoch_time = 0;
      imu_time_char.readValue(&epoch_time, sizeof(time_t));
      Serial.println(epoch_time);

      // delay(20);
    }

    BLE.scanForName(ble_gateway_name);
  }
#endif

  // delay(2000);

  // LowPower.sleep(2000);
  // myPowSave.
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
  Serial.print("epoch now: ");
  Serial.println(imu_data.epoch_time);
  if (now.year() < 2023) {
    send_to_ble();
  }
  // Serial.print("epoch time: ");
  // Serial.println(imu_data.epoch_time);

  return &imu_data;
}

static void __print_buffer(uint8_t *data_buffer, uint32_t len) {
  bool break_val = 0;
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 16; j++) {
#if 0
      if (i * 16 + j < len) {
      // Serial.println("");
      Serial.print("Size: ");
      Serial.print(len);
      Serial.print(", Pointer: ");
      Serial.print(i * 16 + j);
      Serial.print(", Value: ");
        Serial.println(data_buffer[i * 16 + j]);
      } else {
        break_val = 1;
        break;
      }

      if (break_val) break;
#endif
#if 1
      if (i * 16 + j < len) {
        char prnt_buffer[8];
        uint8_t x = data_buffer[i * 16 + j];
        if (x < 10) Serial.print("0");
        if (x < 100) Serial.print("0");

        if (j < 15) {
          snprintf(prnt_buffer, sizeof(prnt_buffer), "%d, ", x);
          Serial.print(prnt_buffer);
        } else {
          snprintf(prnt_buffer, sizeof(prnt_buffer), "%d", x);
          Serial.println(prnt_buffer);
        }
      } else {
        break_val = 1;
        break;
      }
#endif
    }

    if (break_val) break;
  }
  Serial.println("");
}

static uint8_t imu_sector_buffer[4096];

static int __read_sector(uint32_t sector_number, uint8_t *sector_buffer) {
  int ret = 0;
  Serial.print("Sector number: ");
  Serial.println(sector_number);
  Serial.print("Page number: ");
  Serial.println(sector_number * 16);

  if (flash.readByteArray(sector_number * 16, sector_buffer, 4096)) {
    Serial.println("Flash read Success");
    __print_buffer(sector_buffer, flash_header.addr);
    ret = 1;
  } else {
    Serial.println("Flash read failed");
  }

  return ret;
}


static int __write_data_to_flash(s_imu_data_t *__imu_data) {
  if (__read_sector(flash_header.sector, imu_sector_buffer)) {
    if (flash.eraseSector(flash_header.sector)) {
      Serial.println("Flash erase sector success");
      memcpy(imu_sector_buffer + ((flash_header.page % 16) + flash_header.addr), (uint8_t *)__imu_data, sizeof(s_imu_data_t));
      if (flash.writeByteArray(flash_header.page, imu_sector_buffer, sizeof(imu_sector_buffer))) {
        Serial.println("Flash write success");
        __print_buffer(imu_sector_buffer, flash_header.addr + sizeof(s_imu_data_t));
      } else {
        Serial.println("Flash write failed!");
      }
    } else {
      Serial.println("Flash erase failed!");
    }
  } else {
    Serial.println("Sector read failed!");
  }
#if 0
  if (__imu_data) {
    if (flash.readByteArray(flash_header.page, imu_sector_buffer, sizeof(imu_sector_buffer))) {
      memcpy(imu_sector_buffer + flash_header.addr, (uint8_t *)__imu_data, sizeof(s_imu_data_t));

      //   Serial.println("Write Buffer: ");
      // __print_buffer(imu_sector_buffer);
      if (flash.writeByteArray(flash_header.page, imu_sector_buffer, sizeof(imu_sector_buffer))) {
        memset(imu_sector_buffer, 0, sizeof(imu_sector_buffer));
        flash.eraseSector(0);
        // flash.write
        if (flash.readByteArray(flash_header.page, imu_sector_buffer, sizeof(imu_sector_buffer))) {
          // Serial.print("Read Buffer: ");
          // __print_buffer(imu_sector_buffer);
          s_imu_data_t imu_data;
          memcpy(&imu_data, imu_sector_buffer, sizeof(s_imu_data_t));
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
        }
      }
    }
  }
#endif
}

static int __update_flash_header(void) {
  Serial.print("flash_header.sector: ");
  Serial.println(flash_header.sector);
  Serial.print("flash_header.page: ");
  Serial.println(flash_header.page);
  Serial.print("flash_header.addr: ");
  Serial.println(flash_header.addr);
  Serial.print("flash_header.count: ");
  Serial.println(flash_header.count);

  if (flash_header.addr + sizeof(s_imu_data_t) > FLASH_PAGE_SIZE_BYTES) {
    if (flash_header.page < flash_max_page) {
      flash_header.page += 1;
      flash_header.sector = flash_header.page / 16;

      flash_header.addr = 0;
      flash_header.count += 1;
    } else {
      Serial.println("Warnnig: Memory is full!");
      flash.eraseSector(0);
      flash_header.sector = 1;
      flash_header.page = 16;
      flash_header.addr = 0;
      flash_header.magic = FLASH_MAGIC_BYTES;
      flash.writeByteArray(0, (uint8_t *)&flash_header, sizeof(s_flash_header_t));
    }
  } else {
    flash_header.count += 1;
    flash_header.addr += sizeof(s_imu_data_t);
  }

  if (flash.eraseSector(0)) {
    flash_header.magic = FLASH_MAGIC_BYTES;
    if (flash.writeByteArray(0, (uint8_t *)&flash_header, sizeof(s_flash_header_t))) {
      Serial.println("Flash header updated");
    } else {
      Serial.println("Flash header write failed!");
    }
  } else {
    Serial.println("Flash header erase sector failed!");
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
