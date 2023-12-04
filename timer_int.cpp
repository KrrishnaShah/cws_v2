#include "timer_int.h"
#include "NRF52_MBED_TimerInterrupt.h"
#include <mbed.h>

using namespace rtos;

#if (1 == TEST_MODE)
static bool toggle0 = false;
#endif

static volatile uint32_t preMillisTimer0 = 0;

static Semaphore timer_semaphore(1);
static NRF52_MBED_Timer timer_1(NRF_TIMER_3);

static void __interrupt_handler(void);

void timer_int_wait(void) {
  timer_semaphore.acquire();
}

void timer_int_init(void) {
  timer_semaphore.release();
#if (1 == TEST_MODE)
  pinMode(LED_BUILTIN, OUTPUT);
#endif

  // Interval in microsecs
  if (timer_1.attachInterruptInterval(TIMER0_INTERVAL_MS * 1000, __interrupt_handler)) {
    TRACE("Starting timer_1 OK, millis() = ");
  } else {
    TRACE("Can't set timer_1. Select another freq. or timer");
  }
}

static void __interrupt_handler(void) {
  // timer interrupt toggles pin LED_BUILTIN
  timer_semaphore.release();
#if (1 == TEST_MODE)
  digitalWrite(LED_BUILTIN, toggle0);
  toggle0 = !toggle0;
#endif
}