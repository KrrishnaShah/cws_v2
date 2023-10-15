#ifndef __TRACE_H__
#define __TRACE_H__

#include "config.h"

#ifndef __FILENAME__
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#endif

#if (1 == ENABLE_DEBUG)
#define TRACE(X, REG...)                          \
    {                                             \
        printf("%s:%d > " X "\r\n", __FILENAME__, __LINE__, ##REG); \
    }
#else
#define TRACE(X, REG...)
#endif

#define trace_init() Serial1.begin(250000);

#endif // __TRACE_H__
