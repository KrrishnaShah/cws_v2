#ifndef __RTC_APP_H__
#define __RTC_APP_H__
#include "stdint.h"

void rtc_app_init(void);
uint32_t rtc_app_get_time_now(void);
void rtc_app_adjust_time(uint32_t epoch_now);

#endif // __RTC_APP_H__
