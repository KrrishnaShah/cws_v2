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

#include <Arduino.h>
#include <LSM6DS3.h>
#include <ArduinoBLE.h>
// #include <ArduinoLowPower.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_I2CDevice.h>
#include <RTClib.h>
#include <SPIMemory.h>
#include <mbed.h>
#include <Wire.h>
#include <UUID.h>
#include "imu_buffer_link_list.h"

using namespace mbed;
using namespace rtos;
using namespace std::chrono_literals;


#define TEMP_POWER D3
#define IMU_I2C_ADDRESS 0x6A

#define DEVICE_NAME "Device-3"
#define DEVICE_NAME_LEN 16
#define BLE_MTU_SIZE (sizeof(s_imu_data_t) * 7)

#define MAX_SAMPLE_COUNT 800
#define BLE_START_TRANMISSION_COUNT 200
#define BLE_RUN_TRANMISSION_UNTIL_COUNT 20

static uint32_t imu_data_count = 0;

static const char *cws_data_char_uuid = "e202";
static const char *cws_time_char_uuid = "e203";
static const char *ble_gateway_name = "cws-gateway";

static s_imu_data_t imu_data;

static RTC_PCF8563 rtc_pcf8563;
static LSM6DS3 myIMU(I2C_MODE, IMU_I2C_ADDRESS);  // I2C device address 0x6A

static Thread ble_thread;
static Thread sensor_thread;

static void __blinky(long blinky_ms);
static void __updateAccelerometer(void);
static void __print_buffer(uint8_t *buffer, uint32_t len);

static void ble_thread_process(void);
static void sensor_thread_process(void);
static void __config_pedometer(bool clear_step_count);

static void __imu_data_write_to_link_list(s_imu_data_t *new_imu_data);
static uint32_t __imu_data_read_from_link_list(uint8_t *data, uint32_t count);

void setup() {
  pinMode(TEMP_POWER, OUTPUT);   // initialize the built-in LED pin to indicate when a central is connected
  pinMode(LED_BUILTIN, OUTPUT);  // initialize the built-in LED pin to indicate when a central is connected

  digitalWrite(TEMP_POWER, HIGH);
  Serial.begin(250000);

  NRF_WDT->CONFIG = 0x01;           // Configure WDT to run when CPU is asleep
  NRF_WDT->CRV = (10 * 32768) + 1;  // Timeout set to 120 seconds, timeout[s] = (CRV-1)/32768
  NRF_WDT->RREN = 0x01;             // Enable the RR[0] reload register
  NRF_WDT->TASKS_START = 1;         // Start WDT

  if (myIMU.begin() != 0) {
    Serial.println("Accelerometer error");
  } else {
    __config_pedometer(false);
  }

  rtc_pcf8563.begin();
  if (rtc_pcf8563.isrunning()) {
    // Serial.println("RTC is running.");
  } else {
    Serial.println("RTC is NOT running!");
    // rtc_pcf8563.adjust(DateTime(__DATE__, __TIME__));
  }

  rtc_pcf8563.adjust(DateTime(2023, 1, 1, 0, 0, 0));
  Serial.print("Size of data: ");
  Serial.println(sizeof(s_imu_data_t));

  sensor_thread.start(sensor_thread_process);
  ble_thread.start(ble_thread_process);
}

static void ble_thread_process(void) {
  Serial.println("BLE thread is running");
  for (;;) {
    if (imu_buffer_data_count() > BLE_START_TRANMISSION_COUNT) {
      Serial.print(__LINE__);
      Serial.print(": BLE-thread:: Ring buffer read_pointer: ");
      Serial.println(imu_buffer_data_count());

      send_to_ble_from_link_list();
    }

    ThisThread::sleep_for(200);
  }
}

static void sensor_thread_process(void) {
  Serial.println("Sensor thread is running");
  for (;;) {
    __updateAccelerometer();
    __update_pedometer();
    __imu_data_write_to_link_list(&imu_data);
    __print_buffer((uint8_t *)&imu_data, sizeof(s_imu_data_t));
    Serial.println("");
    // ThisThread::sleep_for(35ms);
    ThisThread::sleep_for(185ms);
  }
}


void loop() {
  static int led_state;
  led_state ^= 1;
  digitalWrite(LED_BUILTIN, led_state);
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;
  delay(500);
}

static void __update_pedometer(void) {
  uint8_t dataByte = 0;
  uint16_t stepCount = 0;

  myIMU.readRegister(&dataByte, LSM6DS3_ACC_GYRO_STEP_COUNTER_H);
  stepCount = (dataByte << 8) & 0xFFFF;

  myIMU.readRegister(&dataByte, LSM6DS3_ACC_GYRO_STEP_COUNTER_L);
  stepCount |= dataByte;

  imu_data.pedometer = stepCount;
  Serial.print("Pedometer: ");
  Serial.println(stepCount);
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

  DateTime now = rtc_pcf8563.now();
  imu_data.epoch_time = now.unixtime();

  Serial.print(": ");
  Serial.println(imu_data.epoch_time);
  Serial.print("Data in link list: ");
  Serial.println(imu_buffer_data_count());

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
}

static void __print_buffer(uint8_t *data_buffer, uint32_t len) {
  bool break_val = 0;
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 16; j++) {
      if (i * 16 + j < len) {
        char prnt_buffer[8];
        uint8_t x = data_buffer[i * 16 + j];
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

static int send_to_ble_from_link_list(void) {
  int ret = 0;
  if (BLE.begin()) {
    if (BLE.scanForName(ble_gateway_name)) {
      ThisThread::sleep_for(50ms);

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

            while (imu_buffer_data_count() >= BLE_RUN_TRANMISSION_UNTIL_COUNT) {
              bool write_success = 0;
              uint32_t item_read = __imu_data_read_from_link_list(ble_buffer + DEVICE_NAME_LEN, 7);
              if (item_read) {
                Serial.println("265: Sending to ble: ");
                __print_buffer(ble_buffer, (item_read * sizeof(s_imu_data_t)) + DEVICE_NAME_LEN);

                uint32_t retries = 5;
                while (retries-- && (false == write_success)) {
                  Serial.print("269: BLE: data write faield - ");
                  Serial.println(retries);
                  write_success = imu_data_char.writeValue(ble_buffer, (item_read * sizeof(s_imu_data_t) + DEVICE_NAME_LEN));
                }

                if (!peripheral.connected()) {
                  break;
                }
              } else {
                Serial.println("278 - __ring_buffer_read failed!");
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
              Serial.println("294 - Ble time read failed!");
            }

            peripheral.disconnect();
            Serial.println(epoch_time);
          } else {
            Serial.println("300 - BLE connection failed!");
          }
        } else {
          Serial.println("303 - Failed to connect!");
        }
      } else {
        Serial.println("\r\nError: peripheral not found!\r\n");
      }

      if (peripheral.connected()) {
        peripheral.disconnect();
      }

      BLE.stopScan();
    }

    BLE.end();
  }

  return ret;
}

static void __imu_data_write_to_link_list(s_imu_data_t *new_imu_data) {
  if (imu_buffer_data_count() >= MAX_SAMPLE_COUNT) {
    l_link_list_t *tmp_data = imu_buffer_link_list_pop();
    if (tmp_data) {
      free(tmp_data);
      // Serial.println()
    }
  }

  if (0 == imu_buffer_link_list_push(new_imu_data)) {
    Serial.println("Malloc Failed!");
    while (1) {}
  }
}

static uint32_t __imu_data_read_from_link_list(uint8_t *data, uint32_t count) {
  uint32_t ret = 0;
  for (int idx = 0; idx < count; idx++) {
    l_link_list_t *tmp_imu_data = imu_buffer_link_list_pop();
    if (tmp_imu_data) {
      memcpy(data + (idx * sizeof(s_imu_data_t)), &tmp_imu_data->imu_data, sizeof(s_imu_data_t));
      free(tmp_imu_data);
      tmp_imu_data = NULL;
      ret += 1;
    } else {
      break;
    }
  }

  return ret;
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

static void __config_pedometer(bool clear_step_count) {
  uint8_t errorAccumulator = 0;
  uint8_t dataToWrite = 0;  //Temporary variable

  //Setup the accelerometer******************************
  dataToWrite = 0;

  //  dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
  dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
  dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_26Hz;


  // Step 1: Configure ODR-26Hz and FS-8g
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

  // Step 2: Set bit Zen_G, Yen_G, Xen_G, FUNC_EN, PEDO_RST_STEP(1 or 0)
  if (clear_step_count) {
    myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL10_C, 0x3E);
  } else {
    myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL10_C, 0x3C);
  }

  // Step 3:	Enable pedometer algorithm
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x40);

  //Step 4:	Step Detector interrupt driven to INT1 pin, set bit INT1_FIFO_OVR
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_INT1_CTRL, 0x10);
}