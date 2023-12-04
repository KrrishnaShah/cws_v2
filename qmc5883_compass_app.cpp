#include <DFRobot_QMC5883.h>
#include "trace.h"
#include "config.h"
#include "qmc5883_compass_app.h"

DFRobot_QMC5883 compass(&Wire, QMC5883_ADDRESS);

void qmc5883_compass_app_get_value(s_imu_data_t *imu_data)
{
    if (imu_data)
    {
        sVector_t mag = compass.readRaw();
        imu_data->mag_val_x = mag.XAxis;
        imu_data->mag_val_y = mag.YAxis;
        imu_data->mag_val_z = mag.ZAxis;

        TRACE("mag-x: %d, mag-y: %d, mag-z: %d", mag.XAxis, mag.YAxis, mag.ZAxis);
        TRACE("mag-x: %d, mag-y: %d, mag-z: %d", imu_data->mag_val_x, imu_data->mag_val_y, imu_data->mag_val_z);
    }
}

void qmc5883_compass_app_init(void)
{
    if (!compass.begin())
    {
        TRACE("Compass initialization failed!");
    }
    else
    {
        static const float declination_angle = (COMPASS_DECLINATION_ANGLE_DEGREE + (COMPASS_DECLINATION_ANGLE_MINUTE / 60.0)) / (180 / PI);
        compass.setDeclinationAngle(declination_angle);
        compass.setMeasurementMode(QMC5883_CONTINOUS);
        compass.setDataRate(QMC5883_DATARATE_50HZ);
        TRACE("Compass configuration done.");
    }
}
