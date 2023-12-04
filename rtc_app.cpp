#include "avr/pgmspace.h"
#include <sys/_stdint.h>
#include "trace.h"
#include "mbed.h"
#include "rtc_app.h"
#include "pins_arduino.h"

using namespace rtos;
using namespace std::chrono_literals;
using namespace mbed;

#if (SELECTED_RTC == MCP7940_RTC)

#include <MCP7940.h>

static MCP7940_Class MCP7940;

int rtc_app_read_ram(uint8_t *read_buf, uint32_t len)
{
  int ret = 0;
  if (read_buf)
  {
    uint8_t tmp_buffer[1];
    if (MCP7940.readRAM(0, tmp_buffer))
    {
      memcpy(read_buf, tmp_buffer, len < sizeof(tmp_buffer) ? len : sizeof(tmp_buffer));
      ret = 1;
    }
  }
  return ret;
}

int rtc_app_write_ram(uint8_t *read_buf, uint32_t len)
{
  int ret = 0;
  if (read_buf)
  {
    uint8_t tmp_buffer[64];
    memcpy(tmp_buffer, read_buf, len < sizeof(tmp_buffer) ? len : sizeof(tmp_buffer));
    if (MCP7940.writeRAM(0, tmp_buffer))
    {
      ret = 1;
    }
  }
  return ret;
}

void rtc_app_adjust_time(uint32_t epoch_now)
{
  if (epoch_now > 1693807481)
  {
    TRACE("timer raeding: %d", epoch_now);
    MCP7940.adjust(DateTime(epoch_now));
    ThisThread::sleep_for(5000ms);
  }
  else
  {
    TRACE("ERROR: timer raeding failed: %d", epoch_now);
  }
}

uint32_t rtc_app_get_time_now(void)
{
  DateTime now = MCP7940.now();
  return now.unixtime();
}

void rtc_app_init(void)
{

  TRACE("Here");
  if (!MCP7940.begin()) // (PIN_WIRE_SDA, PIN_WIRE_SCL))
  {
    TRACE("MCP7940(RTC) is NOT running!");
  }
  TRACE("Here");

  Serial.println(F("MCP7940(RTC) initialized."));

  while (!MCP7940.deviceStatus())
  { // Turn oscillator on if necessary
    TRACE("Oscillator is off, turning it on.");
    bool deviceStatus = MCP7940.deviceStart(); // Start oscillator and return state
    if (!deviceStatus)
    {                                                   // If it didn't start
      TRACE("Oscillator did not start, trying again."); // Show error and
      delay(1000);                                      // wait for a second
    }
    else
    { // of if-then oscillator didn't start
      TRACE("Oscillator is running ...");
    }
  }

  // MCP7940.adjust();  // Set to library compile Date/Time
}

#elif (SELECTED_RTC == PCF8563_RTC)
#include <RTClib.h>

static RTC_PCF8563 rtc_pcf8563;

void rtc_app_adjust_time(uint32_t epoch_now)
{
  if (epoch_now > 1693807481)
  {
    TRACE("timer raeding: %d", epoch_now);
    rtc_pcf8563.adjust(DateTime(epoch_now));
  }
  else
  {
    TRACE("ERROR: timer raeding failed: %d", epoch_now);
  }
}

uint32_t rtc_app_get_time_now(void)
{
  DateTime now = rtc_pcf8563.now();
  return now.unixtime();
}

void rtc_app_init(void)
{
  TRACE("Here");
  rtc_pcf8563.begin();
  if (!rtc_pcf8563.isrunning())
  {
    TRACE("RTC is NOT running!");
  }
  rtc_pcf8563.adjust(DateTime(2023, 1, 1, 0, 0, 0));
}
#endif
