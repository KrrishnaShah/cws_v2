// #include <Adafruit_I2CDevice.h>
#include "stdlib.h"
#include "trace.h"
#include "rtc_app.h"

#if (SELECTED_RTC == MCP7940_RTC)

#include <MCP7940.h>

static MCP7940_Class mcp7940_rtc;

void rtc_app_adjust_time(uint32_t epoch_now) {
  if (epoch_now > 1693807481) {
    TRACE("timer raeding: %d", epoch_now);
    mcp7940_rtc.adjust(DateTime(epoch_now));
  } else {
    TRACE("ERROR: timer raeding failed: %d", epoch_now);
  }
}

uint32_t rtc_app_get_time_now(void) {
  return 0;
}

void rtc_app_init(void) {
  if (!mcp7940_rtc.begin()) {
    TRACE("Failed to init rtc!");
  }
}

#elif (SELECTED_RTC == PCF8563_RTC)

#include <RTClib.h>
static RTC_PCF8563 rtc_pcf8563;

void rtc_app_adjust_time(uint32_t epoch_now) {
  if (epoch_now > 1693807481) {
    TRACE("timer raeding: %d", epoch_now);
    rtc_pcf8563.adjust(DateTime(epoch_now));
  } else {
    TRACE("ERROR: timer raeding failed: %d", epoch_now);
  }
}


uint32_t rtc_app_get_time_now(void) {
  DateTime now = rtc_pcf8563.now();
  return now.unixtime();
}


void rtc_app_init(void) {
  TRACE("Here");
  rtc_pcf8563.begin();
  if (!rtc_pcf8563.isrunning()) {
    TRACE("RTC is NOT running!");
  }
  rtc_pcf8563.adjust(DateTime(2023, 1, 1, 0, 0, 0));
}
#endif