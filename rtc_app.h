#ifndef __RTC_APP_H__
#define __RTC_APP_H__

#define PCF8563_RTC 1
#define MCP7940_RTC 2

#define SELECTED_RTC MCP7940_RTC

void rtc_app_init(void);
uint32_t rtc_app_get_time_now(void);
void rtc_app_adjust_time(uint32_t epoch_now);
int rtc_app_read_ram(uint8_t *read_buf, uint32_t len);
int rtc_app_write_ram(uint8_t *read_buf, uint32_t len);

#endif // __RTC_APP_H__
