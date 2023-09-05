#ifndef __IMU_BUFFER_LINK_LIST_H__
#define __IMU_BUFFER_LINK_LIST_H__

#ifdef __cplusplus
extern "C"
{
#endif


#include "string.h"
#include "stdlib.h"
#include "stdint.h"

typedef struct s_imu_data {
  uint32_t sn;
  uint32_t epoch_time;

  int16_t acc_val_x;
  int16_t acc_val_y;
  int16_t acc_val_z;

  int16_t gyro_val_x;
  int16_t gyro_val_y;
  int16_t gyro_val_z;

  uint16_t pedometer;
  uint16_t battery;
} s_imu_data_t __attribute__ ((__packed__));

typedef struct l_link_list {
  s_imu_data_t imu_data;
  struct l_link_list *next;
  struct l_link_list *prev;
} l_link_list_t;

uint32_t imu_buffer_link_list_push(s_imu_data_t* imu_data);
// void imu_buffer_link_list_push(s_imu_data_t *imu_data);
l_link_list_t* imu_buffer_link_list_pop(void);
uint32_t imu_buffer_data_count(void);


#ifdef __cplusplus
}
#endif

#endif  // __IMU_BUFFER_LINK_LIST_H__
