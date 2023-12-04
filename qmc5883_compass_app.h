#ifndef __QMC5883_COMPASS_APP_H__
#define __QMC5883_COMPASS_APP_H__
#include "imu_buffer_link_list.h"

void qmc5883_compass_app_init(void);
void qmc5883_compass_app_get_value(s_imu_data_t *imu_data);

#endif // __QMC5883_COMPASS_APP_H__
