// FastPwmPin - Arduino library to enable fast PWM on an output pin
// For documentation see https://github.com/maxint-rd/FastPwmPin
//
// Support for fast PWM is depending on the used MCU.
// On ATtiny85 Timer1 is used for fast PWM. Timer0 is reserved for the delay() and millis() functions.
// Using the internal oscillator the ATtiny85 can be set to a 1 MHz, 8 Mhz or 16 Mhz system clock.
// Using the 64 MHz prescaler, the generated frequency can be higher than the system clock.
// 
// On ATtiny85 this library only supports pin 4 and pin 1 (0 and 3 partly supported as inverted PWM)
//
//
//
//
//   Pinout ATtiny13A
//                                        +---v---+
//          (PCINT5/!RESET/ADC0/dW) PB5 --|1     8|-- VCC
//               (PCINT3/CLKI/ADC3) PB3 --|2     7|-- PB2 (SCK/ADC1/T0/PCINT2)
//                    (PCINT4/ADC2) PB4 --|3     6|-- PB1 (MISO/AIN1/OC0B/INT0/PCINT1)
//                                  GND --|4     5|-- PB0 (MOSI/AIN0/OC0A/PCINT0)
//                                        +-------+
//  ATtiny13A Fast PWM pins: PB0=OC0A, PB1=OC0B
//

//
//   Pinout ATtiny85
//                                        +---v---+
//          (PCINT5/!RESET/ADC0/dW) PB5 --|1     8|-- VCC
//   (PCINT3/XTAL1/CLK1/!OC1B/ADC3) PB3 --|2     7|-- PB2 (SCK/USCK/SCL/ADC1/T0/INT0/PCINT2)
//    (PCINT4/XTAL2/CLK0/OC1B/ADC2) PB4 --|3     6|-- PB1 (MISO/DO/AIN1/OC0B/OC1A/PCINT1)
//                                  GND --|4     5|-- PB0 (MOSI/DI/SDA/AIN0/!OC0A/AREF/PCINT0)
//                                        +-------+
//  ATtiny85 Fast PWM pins: PB0 (D0)=!OC0A, PB1 (D1)=OC0B/OC1A, PB3 (D4)=!OC1B, PB4 (D3)=OC1B
//  Note: Arduino pin 3 is PB4 and pin 4 is PB3
//

//  ATmega168 Fast PWM pins: PD3(D3)=OC2B, PD5(D5)=OC0B, PD6(D6)=OC0A, PB1(D9)=OC1A, PB2(D10)=OC1B, PB3(D11)=OC2A, 
//  Timers: TC0:8-bit, TC1:16-bit, TC2:8-bit
//

#include "FastPwmPin.h"

#if defined (ARDUINO_ARCH_ESP8266)
#error FastPwmPin does not support ESP8266 (yet)
#endif

int FastPwmPin::enablePwmPin(const int nPreferredPin, unsigned long ulFrequency, uint8_t nPeriodPercentage)		//  nPreferredPin=0
{	// Enable FastPwm on the desired pin
	// Note: since not all MCU's support fast PWM on every pin, the nPreferredPin indicates preference
	// Depending on the MCU a different pin may actually be enabled, pin number is returned
	// Supported frequencies:
	// ATmega168@8MHz: 31.25kHz (31250L) - 4.00MHz (4000000L)
	// The period (duty-cycle) percentage 1-99%. When larger than 50% the output mode is inverting. The resolution is lower at higher frequencies.
	
#if defined (ARDUINO_ARCH_ESP8266)
	if(nFrequency!=0)
  	analogWriteFreq(ulFrequency); // Note: analogWriteFreq(0);  gives a spontaneous WDT reset
  analogWrite(nPreferredPin, 1024/(100/nPeriodPercentage);  // default range is 1024, use 0 to start quiet using pulse-width zero
  return(nPreferredPin);
#elif defined(__AVR_ATmega168P__)  ||  defined (__AVR_ATmega168__) || defined (__AVR_ATmega328P__) ||  defined (__AVR_ATmega328__)
	//
	//			ATmega328/168
	//

	// ATtiny168@8MHz/3v3 can generate 31.25kHz - 4.0MHz fast PWM
  // ATmega328/168 (Uno/Nano/Pro Mini): pin D3 or pin D11 (D11 toggle only)

	// https://www.arduino.cc/reference/en/language/functions/analog-io/analogwrite/
	// PWM pins ATmega168P: 3, 5, 6, 9, 10, and 11.
	// Timer2: D3, D11 
	// Timer1: D9, D10
	// Timer0: D6, D6
	if((nPreferredPin!=11 && nPreferredPin!=3) || nPeriodPercentage>99 || nPeriodPercentage==0)
		return(-1);
	pinMode(nPreferredPin,OUTPUT);          // Set pin to output
	// Pin D3: Timer2, OC2B
	// Pin D11: Timer2, OC2A
	// ATmega168 @ 8MHz:
	// pwm-pulse OCR2B=0x00, OCR2A=0xFF => f=31.25KHz
	// pwm OCR2B=0x7F, OCR2A=0xFF => f=31.25KHz
  // pwm OCR2B=0x07, OCR2A=0x0F => f=499KHz
	// pwm OCR2B=0x03, OCR2A=0x07 => f=999KHz
	// pwm OCR2B=0x02, OCR2A=0x03 => f=1.99MHz (25%)
	// pwm OCR2B=0x01, OCR2A=0x03 => f=2.00MHz (50%)
	// pwm OCR2B=0x01, OCR2A=0x02 => f=2.66MHz (22%-33%)
	// pwm OCR2B=0x00, OCR2A=0x01 => f=4.00MHz (50%)
	if(nPreferredPin==3)
	  TCCR2A = 1<<COM2B1 | (nPeriodPercentage>50)<<COM2B0 | 1<<WGM21 | 1<<WGM20;		// Clear/Set (non-inverting), fast mode 7
	else
	{
		// See https://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM
		// On 168 pin D11 only toggle mode works, at half the frequency: COM2A0 | WGM22, 
		// inverting/non-inverting modes seem not working on pin 11 in fast mode!
		// See also https://electronics.stackexchange.com/questions/49401/cant-set-to-fast-pwm-ocra-mode
		// (proposed solution won't work)
		ulFrequency*=2;		// compensate for half frequency in toggle mode
    TCCR2A = _BV(COM2A0) | _BV(WGM21) | _BV(WGM20);		// toggle mode, fast mode 7
	  //TCCR2A = 1<<COM2A1 | (nPeriodPercentage>50)<<COM2A0 | 1<<WGM21 | 1<<WGM20;		// Clear/Set (non-inverting), fast mode 7
  }
  TCCR2B = 1<<WGM22 | 1<<CS20;	// fast mode 7, div/1
  OCR2A = (F_CPU/ulFrequency)-1; // pwm top, F_CPU/freq -1
  OCR2B = (OCR2A+1)/(100/(nPeriodPercentage>50?(100-nPeriodPercentage):nPeriodPercentage))-1; // pwm bottom, determines duty cycle, for 50%: (top+1)/2-1
	return(nPreferredPin);
#elif defined(__AVR_ATtiny85__)
	//
	//			ATtiny85
	//
	// ATtiny85@1MHz can generate 4.35kHz - 16.16MHz fast PWM (Fast PLL for >500 kHz)
  // ATtiny85: pin D1, D4 (D0/D3 only as inverted for D1/D4, 51%-99%)
	if(nPreferredPin!=4 && nPreferredPin!=3 && nPreferredPin!=1 && nPreferredPin!=0)
		return(-1);
	// http://www.technoblogy.com/show?QVN - waveform generation
  // http://www.technoblogy.com/show?LE0 - four PWMs on ATtiny85
  // https://www.re-innovation.co.uk/docs/fast-pwm-on-attiny85/
  if(nPreferredPin==4 || nPreferredPin==3)
  {
	  TCCR1 = 0<<PWM1A | 0<<COM1A0 | 1<<CS10;
	  GTCCR = 1<<PWM1B | 1<<COM1B1 | (nPeriodPercentage>50)<<COM1B0;			// PWM on B, clear/set (non-inverting) or set-clear (inverting) mode
		// http://www.technoblogy.com/show?LE0 :
		// "There's a bug in current versions of the ATtiny85, and to use inverted mode on OC1B you also have to set COM1A0 to a non-zero value"
		// see also https://electronics.stackexchange.com/questions/97596/attiny85-pwm-why-does-com1a0-need-to-be-set-before-pwm-b-will-work
		// The ATtiny85 datasheet in the Errata (section 27.2.3#4 / page 213) says this about ATtiny45:
		// "Timer Counter1 PWM output OC1B-XOC1B does not work correctly. Only in the case when the control bits, COM1B1 and COM1B0 are in the
		// same mode as COM1A1 and COM1A0, respectively, the OC1B-XOC1B output works correctly."
		if(nPeriodPercentage>50)
			TCCR1 |=  3<<COM1A0;		// fix for bug also present in ATtiny85
		if(nPreferredPin==3) // PWM top/bottom doesn't work on pin D3, only on 4
		  GTCCR = 1<<PWM1B | 0<<COM1B1 | 1<<COM1B0;			// PWM on B, inverted mode
	}
	else
  {
		// http://www.technoblogy.com/show?LE0 :
		// "There's a bug in current versions of the ATtiny85, and to use inverted mode on OC1B you also have to set COM1A0 to a non-zero value"
	  TCCR1 = 1<<PWM1A | 1<<COM1A1  | (nPeriodPercentage>50)<<COM1A0 | 1<<CS10;			// PWM on A, clear/set (non-inverting) or set-clear (inverting) mode
	  GTCCR = 0<<PWM1B | 0<<COM1B0;
		if(nPreferredPin==0) // PWM top/bottom doesn't work on pin D0, only on 1
		  TCCR1 = 1<<PWM1A | 0<<COM1A1  | 1<<COM1A0 | 1<<CS10;			// PWM on A, inverted mode
	}
/*
	// Using the fast 64MHz PLL clock, the ATtiny85 can generate up to 16Mhz clock signal, even on an 1MHz system clock
	// Above 500kHz the fast PLL clock is used, below 500kHz, the prescaled oscillator is used (@F_CPU).
	// ATtiny85@1MHz can generate 4.35kHz - 16.16MHz fast PWM (Fast PLL for >500 kHz)
	// TODO: set CS00:CS02 for lower frequencies
	// On 1MHz ATtiny85:
	// pwm OCR0B=0x7F, OCR0C=0xFF => f=252KHz  / s=3.94KHz
	// pwm OCR0B=0x3F, OCR0C=0x7F => f=504KHz
	// pwm OCR0B=0x1F, OCR0C=0x3F => f=1.01MHz
	// pwm OCR0B=0x0F, OCR0C=0x1F => f=2.02MHz
	// pwm OCR0B=0x07, OCR0C=0x0F => f=4.04MHz (@33%-50%)
	// pwm OCR0B=0x03, OCR0C=0x07 => f=8.08MHz (@33%)
	// pwm OCR0B=0x01, OCR0C=0x03 => f=16.17MHz (@??) / s=252KHz (@33%)
	// pwm OCR0B=0x02, OCR0C=0x03 => f=16.17MHz (@??) / s=252KHz (@50%)
	// pwm OCR0B=0x01, OCR0C=0x02 => f=21.55MHz (multimeter reading)
	// pwm OCR0B=0x00, OCR0C=0x01 => NO PWM!
  OCR1B = 0x07;
  OCR1C = 0x0F; // pwm top, determines duty cycle (should be below top in OCR1B)  0x02
*/
  if(ulFrequency>=500000L)
  {
		#define FASTPWMPIN_TINY85PLL 64000000L
	  OCR1C = (FASTPWMPIN_TINY85PLL/ulFrequency)-1; // pwm top, F_CPU/freq -1 // pwm top

		//  PLLCSR= 1<<PCKE | 1<<PLLE;    // enable ATTiny85 64MHz clock (high speed mode, 64MHz)
	  PLLCSR= 1<<PCKE | 1<<PLLE | 1<<PLOCK;    // enable ATTiny85 64MHz clock (high speed mode+lock, 64MHz)
	  //PLLCSR= 1<<LSM | 1<<PCKE | 1<<PLLE;    // enable ATTiny85 64MHz clock (low speed mode, 32MHz for VCC<2.7V)
	}
	else
	{
		#define FASTPWMPIN_TINY85OSC 8000000L
	  OCR1C = (F_CPU/ulFrequency)-1; // pwm top, F_CPU/freq -1 // pwm top
	}
  if(nPreferredPin==4 || nPreferredPin==3) OCR1B = (OCR1C+1)/(100/(nPeriodPercentage>50?(100-nPeriodPercentage):nPeriodPercentage))-1; // pwm bottom for pin D4, determines duty cycle, for 50%: (top+1)/2-1 (should be below top in OCR1C)
  if(nPreferredPin==1 || nPreferredPin==0) OCR1A = (OCR1C+1)/(100/(nPeriodPercentage>50?(100-nPeriodPercentage):nPeriodPercentage))-1; // pwm bottom for pin D1, determines duty cycle, for 50%: (top+1)/2-1 (should be below top in OCR1C)
	pinMode(nPreferredPin,OUTPUT);          // Set pin to output
  return(nPreferredPin);
#elif defined(__AVR_ATtiny13__)
	//
	//			ATtiny13
	//
	// ATtiny13@9.6MHz/3v3 can generate 39.5kHz - 1.6MHz fast PWM (higher becomes unstable, 5V not tested ok)
	// TODO: set CS00:CS02 for lower frequencies
	if(nPreferredPin!=0 && nPreferredPin!=1)
		return(-1);
  DDRB |= (1 << nPreferredPin); // pinMode(nPreferredPin,OUTPUT);          // Set pin to output
	if(nPreferredPin==0)  // TODO: only pin==1 seems to work ok
	{
	  //TCCR0A = ((1<<WGM00) | (1<<WGM01) | (1 << COM0A1)| ((nPeriodPercentage>50) << COM0A0)) ; // Fast PWM mode 7, Clear OC0A on Compare Match, set OC0A at TOP (non inverting)
		// inverting/non-inverting modes seem not working on pin 0 in fast mode!
		// See also https://electronics.stackexchange.com/questions/49401/cant-set-to-fast-pwm-ocra-mode
		ulFrequency*=2;		// compensate for half frequency in toggle mode
    TCCR0A = 3<<WGM00 | 1<<COM0A0;		// toggle mode, fast mode 7
	}
	else
	  TCCR0A = ((1<<WGM00) | (1<<WGM01) | (1 << COM0B1)| ((nPeriodPercentage>50) << COM0B0)) ; // Fast PWM mode 7, Clear OC0B on Compare Match, set OC0B at TOP (non inverting)
	TCCR0B  = ((1<<WGM02) | (0<<CS02)| (0<<CS01) | (1<<CS00)); //  fast mode 7, 1 divider  (f=37300, ==F_CPU/256)
	
	// pwm OCR0A=0x80, OCR0B=0x7F => f=73.5KHz
	// pwm OCR0A=0x0F, OCR0B=0x05 => f=598Khz (+/-1kHz)
	// pwm OCR0A=0x0D, OCR0B=0x05 => f=686kHz
	// pwm OCR0A=0x0A, OCR0B=0x05 => f=877kHz
	// pwm OCR0A=0x0A, OCR0B=0x08 => f=877.0kHz (+0.6kHz)
	// pwm OCR0A=0x09, OCR0B=0x04 => f=967kHz (+/-2kHz)
	// pwm OCR0A=0x08, OCR0B=0x01 => f=1.074MHz (+/-3kHz)
	// pwm OCR0A=0x06, OCR0B=0x00 => f=1.395MHz (+/-2kHz)
	// pwm OCR0A=0x05, OCR0B=0x04 => f=1.635MHz (+/-3kHz)
	// pwm OCR0A=0x05, OCR0B=0x00 => f=1.635MHz
	// pwm OCR0A=0x04, OCR0B=0x00 => f=1.975MHz (+/-2KHz)
	// pwm OCR0A=0x04, OCR0B=0x03 => f=1.979MHz (+/-2KHz)
	// pwm OCR0A=0x03, OCR0B=0x02 => f=2.50MHz (+/-7kHz)
	// pwm OCR0A=0x02, OCR0B=0x01 => f=3.39MHz
	// pwm OCR0A=0x02, OCR0B=0x00 => f=3.37MHz (+/-10kHz)
	// pwm OCR0A=0x01, OCR0B=0x00 => f=5.10MHz (+/-7kHz)
  OCR0A = (F_CPU/ulFrequency)-1; // pwm top, F_CPU/freq -1 // pwm top, used as BOTTOM for OC0A (D0) in WGM mode 3
  OCR0B = (OCR0A+1)/(100/(nPeriodPercentage>50?(100-nPeriodPercentage):nPeriodPercentage))-1; // pwm bottom for pin D1, determines duty cycle, for 50%: (top+1)/2-1 (should be below top in OCR1A)
  return(nPreferredPin);
#else
	//
	//			Unknown!
	//
/*
	// https://github.com/allenhuffman/MusicSequencerTest/blob/master/MusicSequencerTest.ino

  // Notes about using an Arduino pin to generate the 4Mhz pulse:
  // For the Teensy 2.0, this is how to make a pin act as a 4MHz pulse.
  // I am using this on my Teensy 2.0 hardware for testing. This can be
  // done on other Arduino models, too, but I have only been using my
  // Teensy for this so far. My original NANO prototype is using an
  // external crystal.

  //pinMode(14, OUTPUT); 
  // Turn on toggle pin mode.
  //TCCR1A |= ((1<<COM1A1));
  // Set CTC mode (mode 4), and set clock to be CPU clock
  //TCCR1B |= ((1<<WGM12) | (1<<CS10));
  // Count to one and then reset and count again.  Since CPU is 16MHz,
  // this will divide clock by 2 and action on the pin.  Since we will be
  // toggling the pin, that will divide by 2 again, giving /4 or 4MHz
  //OCR1A = 1;
	
	
	// Make pin 14 be a 14Mhz signal on the Teensy 2.0: 
	#ifdef TEENSY20 
	pinMode(14,OUTPUT); 
	TCCR1A = 0x43; 
	TCCR1B = 0x19; 
	OCR1A = 1; 
	#endif 
*/
    return(-1);
#endif
}

int FastPwmPin::getPwmPin(void)
{
}

