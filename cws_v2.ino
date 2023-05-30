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


static const char *ble_gateway_name = "cws-gateway";
static s_flash_header_t flash_header;
static const char *cws_data_char_uuid = "e202";
// static const char *cws_data_char_uuid = "0000e202-0000-1000-8000-00805f9b34fb";
static const char *cws_time_char_uuid = "0000e203-0000-1000-8000-00805f9b34fb";

RTC_PCF8563 rtc;
LSM6DS3 myIMU(I2C_MODE, IMU_I2C_ADDRESS);  // I2C device address 0x6A
SPIFlash flash(7);

static void __blinky(long blinky_ms);
static s_imu_data_t *__updateAccelerometer(void);

void setup() {
  pinMode(TEMP_POWER, OUTPUT);   // initialize the built-in LED pin to indicate when a central is connected
  pinMode(LED_BUILTIN, OUTPUT);  // initialize the built-in LED pin to indicate when a central is connected

  digitalWrite(TEMP_POWER, HIGH);
  Serial.begin(115200);

  while (!Serial) { delay(10); }

  BLE.begin();
  BLE.scanForName(ble_gateway_name);

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

  rtc.adjust(DateTime(2023, 1, 1, 0, 0, 0));

  if (flash.begin()) {

    Serial.println("Flash is running.");
    Serial.print("flash.getCapacity: ");
    Serial.println(flash.getCapacity());

    Serial.print("flash.getMaxPage: ");
    Serial.println(flash.getMaxPage());

    flash.readByteArray(0, (uint8_t *)&flash_header, sizeof(s_flash_header_t), 0);
    if (FLASH_MAGIC_BYTES != flash_header.magic) {
      flash_header.addr = 0;
      flash_header.magic = 0;
      flash_header.page = 16;
      flash_header.sector = 1;
      // flash.eraseChip();
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
#if 1
  // s_imu_data_t __updateAccelerometer();

  // DateTime now = rtc.now();
  // char buf[100];
  // strncpy(buf, "DD.MM.YYYY hh:mm:ss", 100);
  // Serial.println(now.toString(buf));

  // uint32_t epoch32 = now.unixtime();
  // Serial.print("epoch time now: ");
  // Serial.println(epoch32);

  // DateTime converted_time(epoch32);
  // strncpy(buf, "converted -> DD.MM.YYYY hh:mm:ss", 100);
  // Serial.println(converted_time.toString(buf));

#endif

  BLEDevice peripheral = BLE.available();
  if (peripheral) {
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();

    BLE.stopScan();
    Serial.println("Connecting ...");

    if (peripheral.connect()) {
      Serial.println("Connected");
    } else {
      Serial.println("Failed to connect!");
    }

    // discover peripheral attributes
    Serial.println("Discovering attributes ...");
    if (peripheral.discoverAttributes()) {
      Serial.println("Attributes discovered");
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

    while (peripheral.connected()) {
      s_imu_data_t *tmp_imu_data = __updateAccelerometer();
      imu_data_char.writeValue(tmp_imu_data, sizeof(s_imu_data_t));
      delay(20);
    }

    BLE.scanForName(ble_gateway_name);
  }

  delay(100);

  // LowPower.sleep(2000);
  // myPowSave.
}

s_imu_data_t imu_data;
static s_imu_data_t *__updateAccelerometer(void) {
  memset(&imu_data, 0, sizeof(s_imu_data_t));

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

  DateTime now = rtc.now();
  imu_data.epoch_time = now.unixtime();
  // Serial.print("epoch time: ");
  Serial.println(imu_data.epoch_time);

  return &imu_data;

  // __write_data_to_flash(&imu_data);
  // __update_flash_header();
}

static void __print_buffer(uint8_t *buffer, uint32_t len) {
  for (int i = 0; i < (len % 16) + 1; i++) {
    for (int j = 0; j < 16; j++) {
      if ((i * 16 + j) < len) {
        if (j == 15) {
          Serial.println(buffer[i * 16 + j], 10);
        } else {
          Serial.print(buffer[i * 16 + j], 10);
          Serial.print(" ");
        }
      } else {
        break;
      }
    }
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
    __print_buffer(sector_buffer, 4096);
    ret = 1;
  } else {
    Serial.println("Flash read failed");
  }

  return ret;
}



static int __write_data_to_flash(s_imu_data_t *__imu_data) {
  if (__read_sector(0, imu_sector_buffer)) {
    flash.eraseSector(flash_header.sector);
    memcpy(imu_sector_buffer + ((flash_header.page % 16) + flash_header.addr), (uint8_t *)__imu_data, sizeof(s_imu_data_t));
    if (flash.writeByteArray(flash_header.page, imu_sector_buffer, sizeof(imu_sector_buffer))) {
      Serial.println("Flash write success");
      __print_buffer(imu_sector_buffer, 4096);
    }
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
  // flash_header.addr

  Serial.print("flash_header.page: ");
  Serial.println(flash_header.page);
  Serial.print("flash_header.addr: ");
  Serial.println(flash_header.addr);

  if (flash_header.addr + sizeof(s_imu_data_t) > FLASH_PAGE_SIZE_BYTES) {
    if (flash_header.page < flash.getMaxPage()) {
      flash_header.page += 1;
      if (flash_header.page % 16) {
        flash_header.sector = flash_header.page % 16;
      }
      flash_header.addr = 0;
      flash_header.count += 1;
    } else {
      Serial.println("Warnnig: Memory is full!");
      flash.eraseSector(0);
      flash.writeByteArray(0, (uint8_t *)&flash_header, sizeof(s_flash_header_t));
    }
  } else {
    flash_header.count += 1;
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
