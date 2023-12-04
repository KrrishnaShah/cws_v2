#pragma once

#include "mt6701.h"

typedef struct s_position_sensor_data {
  uint16_t raw_angle;
  float degrees_angle;
  float radians_angle;
  uint8_t output_resolution_UVW;
  uint16_t output_resolution_ABZ;
  MT6701I2COutputType output_type;
  MT6701I2COutputMode output_mode;
  MT6701I2CDirection output_rotation_direction;

} s_position_sensor_data_t;


void position_sensor_app_init(void);
int position_sensor_is_connected(void);
void position_sensor_read_values(void);
void position_sensor_save_new_settings(void);

