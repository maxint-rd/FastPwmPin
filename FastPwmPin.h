#ifndef __FASTPWMPIN_H__
#define __FASTPWMPIN_H__
#include <Arduino.h>

// For the blink example its nice to have LED_BUILTIN defined, also for ATtiny13 and ATtiny85.
// Depending on the core ATtiny44A may already have LED_BUILTIN as 8.
#ifndef LED_BUILTIN
  #if defined(__AVR_ATtiny85__) ||  defined(__AVR_ATtiny13__) ||  defined(__AVR_ATtiny44__)
    #define LED_BUILTIN 2
  #endif
#endif

#define FASTPWMPIN_TOGGLE 50

class FastPwmPin
{
  public:
    FastPwmPin() {};      // constructor
    static int enablePwmPin(const int nPreferredPin=0, unsigned long ulFrequency=0L, uint8_t nPeriodPercentage=FASTPWMPIN_TOGGLE);
  
  private:
    static uint8_t findPrescaler(unsigned long ulFrequency, uint8_t nTimer=0);
    static const uint16_t aPrescale1[];
#if !defined(__AVR_ATtiny13__) && !defined(__AVR_ATtiny85__) && !defined(__AVR_ATtiny45__) && !defined(__AVR_ATtiny25__)
    static const uint16_t aPrescale2[];
#endif
};

#if (defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__))
// ATTiny24A/44A/84A uses Timer0 for Fast PWM on pins 7 and 8. This impacts delay() and millis().
// To mitigate that, the watchdog timer can be used to replace the original delay() and millis().
#define TINYX4_ENABLE_WDTMILLIS 1
#endif
#if defined(TINYX4_ENABLE_WDTMILLIS)
uint32_t wdt_millis();
void wdt_delay(uint16_t ms);
//#define delay(x) wdt_delay(x)
//#define millis(x) wdt_millis(x)
#endif // TINYX4_ENABLE_WDTMILLIS

#endif //__FASTPWMPIN_H__

