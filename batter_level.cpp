#include <stdio.h>
#include <string.h>
#include "Arduino.h"
#include "battery_level.h"
#include "trace.h"

static int battery_level = 0;

static const float bat_full_level = 3.7;
static const float bat_empty_level = 2.0;

int battery_level_get(void)
{
    return battery_level;
}

void battery_level_update(void)
{
    digitalWrite(PIN_VBAT_ENABLE, LOW);
    int battery = analogRead(PIN_VBAT);
    digitalWrite(PIN_VBAT_ENABLE, HIGH);

    // TRACE("adc-value: %d", battery);
    // TRACE("Battery voltage: %f", 0.003395996F * battery);
    battery = map(battery, 0, 1023, 0, 100);

    if (battery != battery_level)
    {
        // TRACE("Battery Level is now: %d %%", battery);
        battery_level = battery;
    }
}

void battery_level_init(void)
{
}