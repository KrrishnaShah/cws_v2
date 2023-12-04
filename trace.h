#ifndef __TRACE_H__
#define __TRACE_H__

#include "config.h"
#include "string.h"
#include "stdint.h"
#include "stdio.h"


void __dump(const char *file_name, uint32_t line, char *buffer_name, void *_buff, uint32_t ofs, uint32_t cnt);

#ifndef __FILENAME__
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#endif

#if (1 == ENABLE_DEBUG)
#define TRACE(X, REG...) \
  { \
    printf("%s:%d > " X "\r\n", __FILENAME__, __LINE__, ##REG); \
  }
#define dump(buffer_name, buffer, offset, count) __dump(__FILENAME__, __LINE__, buffer_name, buffer, offset, count)  // ESP_LOG_BUFFER_HEX_LEVEL
#else
#define TRACE(X, REG...)
#define dump(buffer_name, buffer, offset, count)
#endif

#define trace_init() Serial1.begin(250000);

#endif  // __TRACE_H__
