#include <cstdio>

#include <Arduino.h>
#include <ArduinoBLE.h>
#include "ble_app.h"
#include <mbed.h>
#include "imu_buffer_link_list.h"
#include "rtc_app.h"
#include "trace.h"
#include "battery_level.h"

using namespace mbed;
using namespace rtos;
using namespace std::chrono_literals;

static Thread ble_thread;

static const char *cws_data_char_uuid = "e202";
static const char *cws_time_char_uuid = "e203";
static const char *cws_battery_char_uuid = "2a19";
static const char *ble_gateway_name = "cws-gateway";
// static const char *ble_gateway_address = "F4:C8:8A:F7:DB:F3"; // my pc
static const char *ble_gateway_address = "B8:27:EB:23:D8:12";  // raspberry-pi

static void __ble_thread_process(void);
static int __send_data_to_ble_server(void);

static BLEDevice __connect_ble(void);
static int __disconnect_ble(BLEDevice peripheral);

static int __ble_sync_time(BLEDevice peripheral);
static int __ble_send_battery(BLEDevice peripheral);
static int __ble_send_imu_data(BLEDevice peripheral);

static uint32_t __imu_data_read_from_link_list(uint8_t *data, uint32_t count);

void ble_app_init(void) {
  ble_thread.start(__ble_thread_process);
}

static int __ble_send_imu_data(BLEDevice peripheral) {
  int ret = 0;

  BLECharacteristic imu_data_char = peripheral.characteristic(cws_data_char_uuid);

  if (!imu_data_char) {

    TRACE("Peripheral does not have imu characteristic!");
    peripheral.disconnect();
  } else if (!imu_data_char.canWrite()) {

    TRACE("Peripheral does not have a writable imu characteristic!");
    peripheral.disconnect();
  } else {

    uint8_t ble_buffer[BLE_MTU_SIZE + DEVICE_NAME_LEN] = { 0 };
    snprintf((char *)ble_buffer, DEVICE_NAME_LEN, "%s", DEVICE_NAME);

    while (imu_buffer_data_count() >= BLE_RUN_TRANMISSION_UNTIL_COUNT) {
      int write_success = 0;
      uint32_t item_read = __imu_data_read_from_link_list(ble_buffer + DEVICE_NAME_LEN, BLE_SAMPLE_COUNT);
      if (item_read) {
        uint32_t data_size_to_send = (item_read * 22 + DEVICE_NAME_LEN);
        write_success = imu_data_char.writeValue(ble_buffer, data_size_to_send);
        uint32_t retries = 5;
        while (--retries && (false == write_success)) {
          TRACE("BLE: data write faield - count: ", retries);
          write_success = imu_data_char.writeValue(ble_buffer, data_size_to_send);

          if (!peripheral.connected()) {
            break;
          }
        }

        if (!retries) {
          peripheral.disconnect();
          break;
        }

        if (!peripheral.connected()) {
          break;
        }
      } else {
        TRACE("Error: __ring_buffer_read failed!");
      }

      if (write_success) {
        ret = 1;
      }
    }
  }

  return ret;
}

static int __ble_sync_time(BLEDevice peripheral) {
  int ret = 0;
  BLECharacteristic imu_time_char = peripheral.characteristic(cws_time_char_uuid);

  if (!imu_time_char) {
    printf("%d: Peripheral does not have imu characteristic!\r\n", __LINE__);
  } else if (!imu_time_char.canRead()) {
    printf("Peripheral does not have a readable imu characteristic!\r\n");
  } else {
    uint32_t epoch_time = 0;
    if (imu_time_char.readValue(&epoch_time, 4)) {
      rtc_app_adjust_time(epoch_time);
    } else {
      TRACE("Ble time read failed!");
    }
  }

  return ret;
}

static int __ble_send_battery(BLEDevice peripheral) {
  int ret = 0;
  BLECharacteristic imu_battery_char = peripheral.characteristic(cws_battery_char_uuid);

  if (!imu_battery_char) {

    TRACE("Error: Peripheral does not have imu characteristic!");
  } else if (!imu_battery_char.canWrite()) {

    TRACE("Error: Peripheral does not have a readable imu characteristic!");
  } else {
    int battery_level = battery_level_get();
    imu_battery_char.writeValue(&battery_level, sizeof(int));
    ret = 1;
  }

  return ret;
}

static int __send_data_to_ble_server(void) {
  int ret = 0;
  BLEDevice peripheral = __connect_ble();

  __ble_send_imu_data(peripheral);
  __ble_sync_time(peripheral);
  __ble_send_battery(peripheral);

  __disconnect_ble(peripheral);

  return ret;
}

static BLEDevice __connect_ble(void) {
  BLEDevice peripheral;
  if (BLE.begin()) {
    if (BLE.scanForName(ble_gateway_name)) {
      uint32_t count = 20;
      while (count--) {
        ThisThread::sleep_for(50ms);
        peripheral = BLE.available();
        if (peripheral) {
          BLE.stopScan();
          TRACE("Found BLE Device");

          if (peripheral.connect()) {
            if (!peripheral.discoverAttributes()) {
              TRACE("Attribute discovery failed!");
            }
          }

          break;
        }
      }
    }
  }
  return peripheral;
}

static int __disconnect_ble(BLEDevice peripheral) {
  TRACE("Here");
  int ret = 0;
  if (peripheral) {
    TRACE("Here");
    peripheral.disconnect();
  }
  BLE.stopScan();
  TRACE("Here");
  BLE.end();
  TRACE("Here");

  return ret;
}

static void __ble_thread_process(void) {
  TRACE("BLE thread is running");
  for (;;) {
    if (imu_buffer_data_count() > BLE_START_TRANMISSION_COUNT) {
      TRACE("BLE-thread:: Ring buffer read_pointer: ", imu_buffer_data_count());
      __send_data_to_ble_server();
    }

    ThisThread::sleep_for(200);
  }
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

static uint32_t __imu_data_read_from_link_list(uint8_t *data, uint32_t count) {
  uint32_t ret = 0;
  for (int idx = 0; idx < count; idx++) {
    l_link_list_t *tmp_imu_data = imu_buffer_link_list_pop();
    if (tmp_imu_data) {
      memcpy(data + (idx * 22), &tmp_imu_data->imu_data, 22);
      free(tmp_imu_data);
      tmp_imu_data = NULL;
      ret += 1;
    } else {
      break;
    }
  }

  return ret;
}
