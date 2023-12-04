#include "diagnostics.h"
#include "mbed_power_mgmt.h"
#include <stdio.h>
#include <string.h>
#include "Arduino.h"
#include "battery_level.h"
#include "trace.h"

static int battery_level = 0;

static const float bat_full_level = 3.7;
static const float bat_empty_level = 2.0;
#define VBAT_MV_PER_LBS (0.003395996F)

int battery_level_get(void)
{
    return battery_level;
}

void battery_level_update(void)
{
    int battery = 0; // analogRead(31);

    uint32_t adcCount = analogRead(PIN_VBAT);
    float adcVoltage = adcCount * VBAT_MV_PER_LBS;
    uint32_t mVBat = (uint32_t)((adcVoltage * (1510.0 / 510.0)) * 1000);

    battery = map(mVBat, 3300, 4200, 0, 100);
    battery = (battery > 100) ? 100 : battery;
    battery = (battery < 0) ? 0 : battery;
    battery_level = battery;

#if (1 == TEST_MODE)
    TRACE("ADC-voltage: %.02f", adcVoltage);
    TRACE("Bat voltage(mV): %.02f", mVBat);
    TRACE("Battery percentage: %d %%", battery);
#endif

    if (battery != battery_level)
    {
#if (1 == TEST_MODE)
        TRACE("Battery Level chnaged to: %d %%", battery);
#endif
    }
}

void battery_level_init(void)
{
    pinMode(PIN_VBAT_ENABLE, OUTPUT);
    digitalWrite(PIN_VBAT_ENABLE, LOW);
}
