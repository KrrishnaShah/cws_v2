#include <string.h>
#include <stdlib.h>
#include <Arduino.h>
#include "imu_buffer_link_list.h"

static l_link_list_t *buf_list_head = NULL;
static l_link_list_t *buf_list_tail = NULL;

extern void print_string(char *arg);

static l_link_list_t *imu_buffer_link_list_create(s_imu_data_t *imu_data)
{
  l_link_list_t *new_buf = malloc(sizeof(l_link_list_t));
  if (new_buf)
  {
    memset(new_buf, 0, 22);
    // memcpy(&new_buf->imu_data, imu_data, sizeof(s_imu_data_t));
    memcpy(new_buf->imu_data + IMU_BUF_POS_COUNT, &imu_data->sn, IMU_BUF_LEN_COUNT);
    memcpy(new_buf->imu_data + IMU_BUF_POS_TIME, &imu_data->epoch_time, IMU_BUF_LEN_TIME);

    memcpy(new_buf->imu_data + IMU_BUF_POS_ACC_X, &imu_data->acc_val_x, IMU_BUF_LEN_ACC_X);
    memcpy(new_buf->imu_data + IMU_BUF_POS_ACC_Y, &imu_data->acc_val_y, IMU_BUF_LEN_ACC_Y);
    memcpy(new_buf->imu_data + IMU_BUF_POS_ACC_Z, &imu_data->acc_val_z, IMU_BUF_LEN_ACC_Z);

    memcpy(new_buf->imu_data + IMU_BUF_POS_GYRO_X, &imu_data->gyro_val_x, IMU_BUF_LEN_GYRO_X);
    memcpy(new_buf->imu_data + IMU_BUF_POS_GYRO_Y, &imu_data->gyro_val_y, IMU_BUF_LEN_GYRO_Y);
    memcpy(new_buf->imu_data + IMU_BUF_POS_GYRO_Z, &imu_data->gyro_val_z, IMU_BUF_LEN_GYRO_Z);

    memcpy(new_buf->imu_data + IMU_BUF_POS_MAG_X, &imu_data->mag_val_x, IMU_BUF_LEN_MAG_X);
    memcpy(new_buf->imu_data + IMU_BUF_POS_MAG_Y, &imu_data->mag_val_y, IMU_BUF_LEN_MAG_Y);
    memcpy(new_buf->imu_data + IMU_BUF_POS_MAG_Z, &imu_data->mag_val_z, IMU_BUF_LEN_MAG_Z);

    memcpy(new_buf->imu_data + IMU_BUF_POS_PEDO, &imu_data->pedometer, IMU_BUF_LEN_PEDO);
  }

  return new_buf;
}

uint32_t imu_buffer_link_list_push(s_imu_data_t *imu_data)
{
  uint32_t ret = 0;
  l_link_list_t *new_node = imu_buffer_link_list_create(imu_data);
  if (new_node)
  {
    if (buf_list_head && buf_list_tail)
    {
      ret = 1;
      buf_list_tail->next = new_node;
      buf_list_tail = buf_list_tail->next;
    }
    else
    {
      ret = 1;
      buf_list_head = new_node;
      buf_list_tail = buf_list_head;
    }
  }

  return ret;
}

l_link_list_t *imu_buffer_link_list_pop(void)
{
  l_link_list_t *ret = NULL;
  if (buf_list_head)
  {
    ret = buf_list_head;
    buf_list_head = buf_list_head->next;
  }
  return ret;
}

uint32_t imu_buffer_data_count(void)
{
  uint32_t ret = 0;
  if (buf_list_head && buf_list_tail)
  {
    uint32_t first = 0;
    uint32_t last = 0;
    memcpy(&first, buf_list_tail->imu_data, 4);
    memcpy(&last, buf_list_head->imu_data, 4);
    ret = first - last;
  }
  return ret;
}
