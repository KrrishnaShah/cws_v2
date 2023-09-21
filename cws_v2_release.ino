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
#include "timer_int.h"

using namespace mbed;
using namespace rtos;
using namespace std::chrono_literals;

#define ENABLE_DEBUG 1

#if (1 == ENABLE_DEBUG)
REDIRECT_STDOUT_TO(Serial)
#define TRACE(X, REG...) \
  { printf("%d: " X "\r\n", __LINE__, ##REG); }
#else
#define TRACE(X, REG...)
#endif


#define TEMP_POWER D3
#define IMU_I2C_ADDRESS 0x6A

#define DEVICE_NAME "Cow-x"

#define DEVICE_NAME_LEN 16
#define BLE_SAMPLE_COUNT 9
#define BLE_MTU_SIZE (sizeof(s_imu_data_t) * BLE_SAMPLE_COUNT)
static const uint32_t max_mtu_size = 240;

#define MAX_SAMPLE_COUNT 1600
#define BLE_START_TRANMISSION_COUNT 800
#define BLE_RUN_TRANMISSION_UNTIL_COUNT 20

// #define MAX_SAMPLE_COUNT 800
// #define BLE_START_TRANMISSION_COUNT 20
// #define BLE_RUN_TRANMISSION_UNTIL_COUNT 2

static int battery_level = 0;
static uint32_t imu_data_count = 0;

static const char *cws_data_char_uuid = "e202";
static const char *cws_time_char_uuid = "e203";
static const char *cws_battery_char_uuid = "2a19";
static const char *ble_gateway_name = "cws-gateway";
// static const char *ble_gateway_address = "F4:C8:8A:F7:DB:F3"; // my pc
static const char *ble_gateway_address = "B8:27:EB:23:D8:12";  // raspberry-pi

#define USE_BLE_GATEWAY_NAME 1
#define USE_BLE_GATEWAY_ADDRESS 2
#define USE_BLE_GATEWAY_TYPE USE_BLE_GATEWAY_NAME

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
// pinMode(TEMP_POWER, OUTPUT);   // initialize the built-in LED pin to indicate when a central is connected
// pinMode(LED_BUILTIN, OUTPUT);  // initialize the built-in LED pin to indicate when a central is connected

// digitalWrite(TEMP_POWER, HIGH);
#if (1 == ENABLE_DEBUG)
  Serial.begin(250000);
#endif

  timer_int_init();

#if 1
  NRF_WDT->CONFIG = 0x01;           // Configure WDT to run when CPU is asleep
  NRF_WDT->CRV = (10 * 32768) + 1;  // Timeout set to 120 seconds, timeout[s] = (CRV-1)/32768
  NRF_WDT->RREN = 0x01;             // Enable the RR[0] reload register
  NRF_WDT->TASKS_START = 1;         // Start WDT

  if (myIMU.begin() != 0) {
    TRACE("Accelerometer error");
  } else {
    __config_pedometer(false);
  }

  rtc_pcf8563.begin();
  if (rtc_pcf8563.isrunning()) {
    // TRACE("RTC is running.");
  } else {
    TRACE("RTC is NOT running!");
    // rtc_pcf8563.adjust(DateTime(__DATE__, __TIME__));
  }

  rtc_pcf8563.adjust(DateTime(2023, 1, 1, 0, 0, 0));
  TRACE("Size of data: %d", sizeof(s_imu_data_t));

  sensor_thread.start(sensor_thread_process);
  ble_thread.start(ble_thread_process);
#endif
}

static void ble_thread_process(void) {
  TRACE("BLE thread is running");
  for (;;) {
    if (imu_buffer_data_count() > BLE_START_TRANMISSION_COUNT) {
      TRACE("BLE-thread:: Ring buffer read_pointer: ", imu_buffer_data_count());
      send_to_ble_from_link_list();
    }

    ThisThread::sleep_for(200);
  }
}

static void sensor_thread_process(void) {
  TRACE("Sensor thread is running");
  for (;;) {
    timer_int_wait();
    __updateAccelerometer();
    __update_pedometer();
    __imu_data_write_to_link_list(&imu_data);
    // __print_buffer((uint8_t *)&imu_data, sizeof(s_imu_data_t));
    printf("\r\n");
    // ThisThread::sleep_for(35ms);
    // ThisThread::sleep_for(150ms);
    // ThisThread::sleep_for(1000ms);
  }
}

static const float bat_full_level = 3.7;
static const float bat_empty_level = 2.0;

static void updateBatteryLevel() {
  // int battery = analogRead(A0);

  digitalWrite(PIN_VBAT_ENABLE, LOW);
  int battery = analogRead(PIN_VBAT);
  digitalWrite(PIN_VBAT_ENABLE, HIGH);

  TRACE("adc-value: %d", battery);
  TRACE("Battery voltage: %f", 0.003395996F * battery);
  battery = map(battery, 0, 1023, 0, 100);

  if (battery != battery_level) {
    TRACE("Battery Level is now: %d %%", battery);
    battery_level = battery;
  }
}

static uint32_t count = 0;
void loop() {
  // static int led_state;
  // led_state ^= 1;
  // digitalWrite(LED_BUILTIN, led_state);
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;
  delay(500);
  // if (0 == (count++ % 30))
  {
    updateBatteryLevel();
  }
}

static void __update_pedometer(void) {
  uint8_t dataByte = 0;
  uint16_t stepCount = 0;

  myIMU.readRegister(&dataByte, LSM6DS3_ACC_GYRO_STEP_COUNTER_H);
  stepCount = (dataByte << 8) & 0xFFFF;

  myIMU.readRegister(&dataByte, LSM6DS3_ACC_GYRO_STEP_COUNTER_L);
  stepCount |= dataByte;

  imu_data.pedometer = stepCount;
  TRACE("Pedometer: %d", stepCount);
}

static void __updateAccelerometer(void) {
  TRACE("total data count: %d", imu_data_count);
  imu_data.acc_val_x = (int16_t)(myIMU.readFloatAccelX() * 100);
  imu_data.acc_val_y = (int16_t)(myIMU.readFloatAccelY() * 100);
  imu_data.acc_val_z = (int16_t)(myIMU.readFloatAccelZ() * 100);

  imu_data.gyro_val_x = (int16_t)(myIMU.readFloatGyroX() * 100);
  imu_data.gyro_val_y = (int16_t)(myIMU.readFloatGyroY() * 100);
  imu_data.gyro_val_z = (int16_t)(myIMU.readFloatGyroZ() * 100);

  imu_data.sn = imu_data_count++;

  DateTime now = rtc_pcf8563.now();
  imu_data.epoch_time = now.unixtime();

  imu_data.battery = (int16_t)battery_level;
  TRACE("(%d) Data in link list: %d", imu_data.epoch_time, imu_buffer_data_count());

#if 0
  TRACE("count: %u", imu_data->sn);
  TRACE("acc-x: %f, acc-y: %f, acc-z: %f", imu_data->acc_val_x / 100.0, imu_data->acc_val_y / 100.0, imu_data->acc_val_z / 100.0);
  TRACE("acc-x: %f, acc-y: %f, acc-z: %f", imu_data->gyro_val_x / 100.0, imu_data->gyro_val_y / 100.0, imu_data->gyro_val_z / 100.0);
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

    if (break_val) {
      break;
    }
  }
  printf("\r\n");
}

static int send_to_ble_from_link_list(void) {
  int ret = 0;
  if (BLE.begin()) {
    if (BLE.scanForName(ble_gateway_name)) {
      ThisThread::sleep_for(50ms);
      uint32_t count = 20;
      BLEDevice peripheral = BLE.available();
      while (count--) {
        if (peripheral) {
          TRACE("Found BLE Device");
          break;
        }
        ThisThread::sleep_for(50ms);
        peripheral = BLE.available();
      }

      if (peripheral) {
        BLE.stopScan();

        if (peripheral.connect()) {
          if (!peripheral.discoverAttributes()) {
            TRACE("Attribute discovery failed!");
            // peripheral.disconnect();
          }

#if 0
          TRACE("Number of Services: %d", peripheral.serviceCount());
          for (int i = 0; i < peripheral.serviceCount(); i++) {
            BLEService service = peripheral.service(i);
            TRACE("Service found: %s", service.uuid());
            TRACE("Number of Characterstics: %d", service.characteristicCount());
            for (int j = 0; j < service.characteristicCount(); j++) {
              BLECharacteristic characteristic = service.characteristic(j);
              TRACE("Characteristics found: %s", characteristic.uuid());
            }
          }
#endif

          BLECharacteristic imu_data_char = peripheral.characteristic(cws_data_char_uuid);
          if (!imu_data_char) {
            TRACE("Peripheral does not have imu characteristic!");
            peripheral.disconnect();
          } else if (!imu_data_char.canWrite()) {
            TRACE("Peripheral does not have a writable imu characteristic!");
            peripheral.disconnect();
          }

          BLECharacteristic imu_time_char = peripheral.characteristic(cws_time_char_uuid);
          if (!imu_time_char) {
            printf("%d: Peripheral does not have imu characteristic!", __LINE__);
            peripheral.disconnect();
          } else if (!imu_time_char.canRead()) {
            printf("Peripheral does not have a readable imu characteristic!");
            peripheral.disconnect();
          }

          BLECharacteristic imu_battery_char = peripheral.characteristic(cws_battery_char_uuid);
          if (!imu_time_char) {
            TRACE("Peripheral does not have imu characteristic!");
            peripheral.disconnect();
          } else if (!imu_time_char.canRead()) {
            TRACE("Peripheral does not have a readable imu characteristic!");
            peripheral.disconnect();
          }

          if (peripheral.connected()) {
            uint8_t ble_buffer[BLE_MTU_SIZE + DEVICE_NAME_LEN] = { 0 };
            snprintf((char *)ble_buffer, DEVICE_NAME_LEN, "%s", DEVICE_NAME);

            while (imu_buffer_data_count() >= BLE_RUN_TRANMISSION_UNTIL_COUNT) {
              bool write_success = 0;
              uint32_t item_read = __imu_data_read_from_link_list(ble_buffer + DEVICE_NAME_LEN, BLE_SAMPLE_COUNT);
              if (item_read) {
                write_success = imu_data_char.writeValue(ble_buffer, (item_read * sizeof(s_imu_data_t) + DEVICE_NAME_LEN));
                uint32_t retries = 5;
                while (retries-- && (false == write_success)) {
                  TRACE("BLE: data write faield - count: ", retries);
                  write_success = imu_data_char.writeValue(ble_buffer, (item_read * sizeof(s_imu_data_t) + DEVICE_NAME_LEN));
                }

                if (!peripheral.connected()) {
                  break;
                }
              } else {
                TRACE("343 - __ring_buffer_read failed!");
              }

              if (write_success) {
                ret = 1;
              }
            }

            uint32_t epoch_time = 0;
            if (imu_time_char.readValue(&epoch_time, 4)) {
              if (epoch_time > 1693807481) {
                TRACE("timer raeding: %d", epoch_time);
                rtc_pcf8563.adjust(DateTime(epoch_time));
                ThisThread::sleep_for(5000ms);
              } else {
                TRACE("ERROR: timer raeding failed: %d", epoch_time);
                ThisThread::sleep_for(5000ms);
              }
            } else {
              TRACE("355 - Ble time read failed!");
            }

            imu_battery_char.writeValue(&battery_level, sizeof(int));

            peripheral.disconnect();
            TRACE("epoch_time: ", epoch_time);
          } else {
            TRACE("BLE connection failed!");
          }
        } else {
          TRACE("Failed to connect!");
        }
      } else {
        TRACE("\r\nError: peripheral not found!\r\n");
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
      // TRACE()
    }
  }

  if (0 == imu_buffer_link_list_push(new_imu_data)) {
    TRACE("Malloc Failed!");
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