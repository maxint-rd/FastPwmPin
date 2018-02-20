#ifndef __FASTPWMPIN_H__
#define __FASTPWMPIN_H__
#include <Arduino.h>

// for the blink example its nice to have LED_BUILTIN defined, also for ATtiny13 and ATtiny85
#ifndef LED_BUILTIN
	#if defined(__AVR_ATtiny85__) ||  defined(__AVR_ATtiny13__) ||  defined(__AVR_ATtiny44__)
		#define LED_BUILTIN 2
	#endif
#endif

#define FASTPWMPIN_TOGGLE 50

class FastPwmPin
{
	public:
		FastPwmPin() {};			// constructor
		static int enablePwmPin(const int nPreferredPin=0, unsigned long ulFrequency=0L, uint8_t nPeriodPercentage=FASTPWMPIN_TOGGLE);
}; 
#endif //__FASTPWMPIN_H__

