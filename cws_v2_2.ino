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
#include <Adafruit_MLX90614.h>
#include <Adafruit_I2CDevice.h>
#include <SPIMemory.h>
#include <mbed.h>
#include <Wire.h>
#include "imu_buffer_link_list.h"
#include "timer_int.h"
#include "trace.h"
#include "ble_app.h"
#include "battery_level.h"
#include "rtc_app.h"
#include "mbed_power_mgmt.h"
#include "DeepSleepLock.h"
#include "qmc5883_compass_app.h"

REDIRECT_STDOUT_TO(Serial)

using namespace mbed;
using namespace rtos;
using namespace std::chrono_literals;

static uint32_t imu_data_count = 0;

static LSM6DS3 myIMU(I2C_MODE, IMU_I2C_ADDRESS); // I2C device address 0x6A

static Thread sensor_thread;

static void __blinky(long blinky_ms);
static void __update_imu_data(s_imu_data_t *imu_data);
static void __print_buffer(uint8_t *buffer, uint32_t len);

static void __ble_thread_process(void);
static void __sensor_thread_process(void);
static void __config_pedometer(bool clear_step_count);
static void __update_pedometer(s_imu_data_t *new_imu_data);

static void __imu_data_write_to_link_list(s_imu_data_t *new_imu_data);

void setup()
{
    trace_init();
#if (1 == TEST_MODE)
    delay(10000);
#endif
    TRACE("size of list: %d", sizeof(s_imu_data_t));
    TRACE("Starting ...");
    rtc_app_init();
    battery_level_init();
    qmc5883_compass_app_init();
    timer_int_init();

#if (0 == TEST_MODE)
    NRF_WDT->CONFIG = 0x01;          // Configure WDT to run when CPU is asleep
    NRF_WDT->CRV = (10 * 32768) + 1; // Timeout set to 10 seconds, timeout[s] = (CRV-1)/32768
    NRF_WDT->RREN = 0x01;            // Enable the RR[0] reload register
    NRF_WDT->TASKS_START = 1;        // Start WDT
#endif

    if (myIMU.begin() != 0)
    {
        TRACE("Accelerometer error");
    }
    else
    {
        __config_pedometer(false);
    }

    sensor_thread.start(__sensor_thread_process);
    ble_app_init();
}

static void __sensor_thread_process(void)
{
    TRACE("Sensor thread is running");
    s_imu_data_t imu_data;
    for (;;)
    {
        timer_int_wait();
        qmc5883_compass_app_get_value(&imu_data);
        __update_imu_data(&imu_data);
        __update_pedometer(&imu_data);
        __imu_data_write_to_link_list(&imu_data);
    }
}

void loop()
{
    battery_level_update();

#if (1 == TEST_MODE)
    int battery_level = battery_level_get();
    TRACE("battery level: %d", battery_level);
    uint32_t time_now = rtc_app_get_time_now();
    TRACE("time now: %d", time_now);
#else
    NRF_WDT->RR[0] = WDT_RR_RR_Reload;
#endif

    delay(1000);
}

static void __update_pedometer(s_imu_data_t *new_imu_data)
{
    uint8_t dataByte = 0;
    uint16_t stepCount = 0;

    myIMU.readRegister(&dataByte, LSM6DS3_ACC_GYRO_STEP_COUNTER_H);
    stepCount = (dataByte << 8) & 0xFFFF;

    myIMU.readRegister(&dataByte, LSM6DS3_ACC_GYRO_STEP_COUNTER_L);
    stepCount |= dataByte;

    new_imu_data->pedometer = stepCount;
    // TRACE("Pedometer: %d", stepCount);
}

static void __update_imu_data(s_imu_data_t *imu_data)
{
    imu_data->acc_val_x = (int16_t)(myIMU.readFloatAccelX() * 100);
    imu_data->acc_val_y = (int16_t)(myIMU.readFloatAccelY() * 100);
    imu_data->acc_val_z = (int16_t)(myIMU.readFloatAccelZ() * 100);

    imu_data->gyro_val_x = (int16_t)(myIMU.readFloatGyroX() * 100);
    imu_data->gyro_val_y = (int16_t)(myIMU.readFloatGyroY() * 100);
    imu_data->gyro_val_z = (int16_t)(myIMU.readFloatGyroZ() * 100);

    // qmc5883_compass_app_get_value(imu_data);

    imu_data->sn = imu_data_count++;
    imu_data->epoch_time = rtc_app_get_time_now();

    TRACE("(%d) -> total data count: %d,  Data in link list: %d", imu_data->epoch_time, imu_data_count, imu_buffer_data_count());

#if (1 == TEST_MODE)
    TRACE("count: %u", imu_data->sn);
    TRACE("acc-x: %f, acc-y: %f, acc-z: %f", imu_data->acc_val_x / 100.0, imu_data->acc_val_y / 100.0, imu_data->acc_val_z / 100.0);
    TRACE("gyro-x: %f, gyro-y: %f, gyro-z: %f", imu_data->gyro_val_x / 100.0, imu_data->gyro_val_y / 100.0, imu_data->gyro_val_z / 100.0);
    TRACE("mag-x: %d, mag-y: %d, mag-z: %d", imu_data->mag_val_x, imu_data->mag_val_y, imu_data->mag_val_z);
#endif
}

static void __print_buffer(uint8_t *data_buffer, uint32_t len)
{
    bool break_val = 0;
    for (int i = 0; i < 16; i++)
    {
        for (int j = 0; j < 16; j++)
        {
            if (i * 16 + j < len)
            {
                char prnt_buffer[8];
                uint8_t x = data_buffer[i * 16 + j];
                if (j < 15)
                {
                    snprintf(prnt_buffer, sizeof(prnt_buffer), "%02X, ", x);
                    Serial.print(prnt_buffer);
                }
                else
                {
                    snprintf(prnt_buffer, sizeof(prnt_buffer), "%02X", x);
                    Serial.println(prnt_buffer);
                }
            }
            else
            {
                break_val = 1;
                break;
            }
        }

        if (break_val)
        {
            break;
        }
    }
    printf("\r\n");
}

static void __imu_data_write_to_link_list(s_imu_data_t *new_imu_data)
{
    if (imu_buffer_data_count() >= MAX_SAMPLE_COUNT)
    {
        l_link_list_t *tmp_data = imu_buffer_link_list_pop();
        if (tmp_data)
        {
            free(tmp_data);
        }
    }

    if (0 == imu_buffer_link_list_push(new_imu_data))
    {
        TRACE("Malloc Failed!");
        while (1)
        {
        }
    }
}

static void __blinky(long blinky_ms)
{
    static int _on;
    static long last_blink_ms;

    if (_on)
    {
        long current_ms = millis();
        if ((current_ms - last_blink_ms) > 100)
        {
            digitalWrite(LED_BUILTIN, 1);
            last_blink_ms = current_ms;
            _on = 0;
        }
    }
    else
    {
        long current_ms = millis();
        if ((current_ms - last_blink_ms) > blinky_ms)
        {
            digitalWrite(LED_BUILTIN, 0);
            last_blink_ms = current_ms;
            _on = 1;
        }
    }
}

static void __config_pedometer(bool clear_step_count)
{
    uint8_t errorAccumulator = 0;
    uint8_t dataToWrite = 0; // Temporary variable

    // Setup the accelerometer******************************
    dataToWrite = 0;

    //  dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
    dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
    dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_26Hz;

    // Step 1: Configure ODR-26Hz and FS-8g
    myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

    // Step 2: Set bit Zen_G, Yen_G, Xen_G, FUNC_EN, PEDO_RST_STEP(1 or 0)
    if (clear_step_count)
    {
        myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL10_C, 0x3E);
    }
    else
    {
        myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL10_C, 0x3C);
    }

    // Step 3:	Enable pedometer algorithm
    myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x40);

    // Step 4:	Step Detector interrupt driven to INT1 pin, set bit INT1_FIFO_OVR
    myIMU.writeRegister(LSM6DS3_ACC_GYRO_INT1_CTRL, 0x10);
}
