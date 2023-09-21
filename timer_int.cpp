#include "timer_int.h"
#include "NRF52_MBED_TimerInterrupt.h"
#include <mbed.h>

using namespace rtos;

static bool toggle0 = false;
volatile uint32_t preMillisTimer0 = 0;

Semaphore timer_semaphore(1);
NRF52_MBED_Timer timer_1(NRF_TIMER_3);

static void __interrupt_handler(void);

void timer_int_wait(void)
{
  timer_semaphore.acquire();
}

void timer_int_init(void) {
  timer_semaphore.release();
  pinMode(LED_BUILTIN, OUTPUT);

  // Interval in microsecs
  if (timer_1.attachInterruptInterval(TIMER0_INTERVAL_MS * 1000, __interrupt_handler)) {
    preMillisTimer0 = millis();
    Serial.print(F("Starting timer_1 OK, millis() = "));
    Serial.println(preMillisTimer0);
  } else
    Serial.println(F("Can't set timer_1. Select another freq. or timer"));
}

static void __interrupt_handler(void) {
  // timer interrupt toggles pin LED_BUILTIN
  timer_semaphore.release();
  digitalWrite(LED_BUILTIN, toggle0);
  toggle0 = !toggle0;
}