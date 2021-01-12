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
//   Pinout ATtiny13A
//                                        +---v---+
//          (PCINT5/!RESET/ADC0/dW) PB5 --|1     8|-- VCC
//               (PCINT3/CLKI/ADC3) PB3 --|2     7|-- PB2 (SCK/ADC1/T0/PCINT2)
//                    (PCINT4/ADC2) PB4 --|3     6|-- PB1 (MISO/AIN1/OC0B/INT0/PCINT1)
//                                  GND --|4     5|-- PB0 (MOSI/AIN0/OC0A/PCINT0)
//                                        +-------+
//  ATtiny13A Fast PWM pins: PB0=OC0A, PB1=OC0B
//

//   Pinout ATtiny24A/44A/84A
//                                         +---v---+
//                                   VCC --|1    14|-- GND
//           (PCINT8/CLKI/XTAL1) D10/PB0 --|2    13|-- PA0/D0 (ADC0/AREF/PCINT0)
//                 (PCINT9/XTAL2) D8/PB1 --|3    12|-- PA1/D1 (ADC1/AIN0/PCINT1)
//               (PCINT11/!RESET/dW) PB3 --|4    11|-- PA2/D2 (ADC2/AIN1/PCINT2)
//      (PCINT10/INT0/OC0A/CKOUT) D8/PB2 --|5    10|-- PA3/D3 (ADC3/T0/PCINT3)
//         (PCINT7/ICP/OC0B/ADC7) D7/PA7 --|6     9|-- PA4/D4 (ADC4/USCK/SCL/T1/PCINT4)
// (PCINT6/OC1A/SDA/MOSI/DI/ADC6) D6/PA6 --|7     8|-- PA5/D5 (ADC5/DO/MISO/OC1B/PCINT5)
//                                         +-------+
//  ATtiny24A/44A/84A PWM pins: PB2(D8)=OC0A, PA7(D7)=OC0B, PA6(D6)=OC1A, PA5(D5)=OC1B
//


//
//   Pinout ATtiny85/45/25
//                                        +---v---+
//          (PCINT5/!RESET/ADC0/dW) PB5 --|1     8|-- VCC
//   (PCINT3/XTAL1/CLK1/!OC1B/ADC3) PB3 --|2     7|-- PB2 (SCK/USCK/SCL/ADC1/T0/INT0/PCINT2)
//    (PCINT4/XTAL2/CLK0/OC1B/ADC2) PB4 --|3     6|-- PB1 (MISO/DO/AIN1/OC0B/OC1A/PCINT1)
//                                  GND --|4     5|-- PB0 (MOSI/DI/SDA/AIN0/OC0A/!OC1A/AREF/PCINT0)
//                                        +-------+
//  ATtiny85 Fast PWM pins: PB0 (D0)=!OC1A, PB1 (D1)=OC1A, PB3 (D4)=!OC1B, PB4 (D3)=OC1B
//  Note currently this library only supports Timer1 on ATtiny85, with pins 0 and 4 as inverted mode pins.
//  Note: Arduino pin 3 is PB4 and pin 4 is PB3
//

//  ATmega168/328 Fast PWM pins: PD3(D3)=OC2B, PD5(D5)=OC0B, PD6(D6)=OC0A, PB1(D9)=OC1A, PB2(D10)=OC1B, PB3(D11)=OC2A, 
//  Timers: TC0:8-bit, TC1:16-bit, TC2:8-bit

//
//  ATmega8A Fast PWM pins: PB1(D9)=OC1A, PB2(D10)=OC1B, PB3(D11)=OC2, 
//  Timers: TC0:8-bit without PWM, TC1:16-bit with PWM, TC2:8-bit with PWM

//  LGT8F328P (SSOP20) Fast PWM up to 16 MHz
//  pins: PC6(RST)=OC3A, PD2(D2)=OC3B, PD3(D3)=OC2B, PD5(D5)=OC0B, PD6(D6)=OC0A/OC3A, PB1(D9)=OC1A, PB2(D10)=OC1B, PB3(D11)=OC2A, PE4()=OC0A, PF1()=OC3A, PF2()=OC3B, PF3()=OC3C/OC0B, PF4()=OC1B, PF5()=OC1A, PF6()=OC2A
//  Timers: TC0:8-bit, TC1:16-bit, TC2:8-bit, TC3:16-bit

//  Support for 16-bit Timer3 on LGT8F328P:
//    QFP48:  OC3A on #15 (PF1) is ??, OC3B on #48 (PF2) is ??, OC3C on PF3 (#4) is ???
//    QFP32L: OC3A on #31 (PF1, shared PD1, alternate function on PD6 via PMX1/PMX2) is D1 (TX), OC3B on #32 (PF2, shared PD2) is D2, OC3C on PF3 is not mapped to pin
//    SSOP20: OC3A on #01 (PF1, shared PC6/RST), OC3B on #02 (PF2, shared PD2) is D2, OC3C on PF3 is not mapped to pin



#include "FastPwmPin.h"

#if defined (ARDUINO_ARCH_ESP8266)
#error FastPwmPin does not support ESP8266 (yet)
#endif

#if defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny25__)
const uint16_t FastPwmPin::aPrescale1[] = {0, 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384};    // prescale settings for ATtiny85/45/25 8-bit Timer1
#else
const uint16_t FastPwmPin::aPrescale1[] = {0, 1, 8, 64, 256, 1024};   // prescale settings for ATmega328/168/8A/etc, 8-bit Timer0 and 16-bit Timer1
#if !defined(__AVR_ATtiny13__)
const uint16_t FastPwmPin::aPrescale2[] = {0, 1, 8, 32, 64, 128, 256, 1024};    // prescale settings for ATmega328/168/8A/etc, 8-bit Timer2
#endif
#endif

uint8_t FastPwmPin::findPrescaler(unsigned long ulFrequency, uint8_t nTimer)
{ // Find the proper prescale setting for the desired frequency
  // Top value (OCR1A, OCR2A) is only valid when F_CPU/prescaler/ulFrequency <= 255 for 8-bit or 65535 for 16-bit
  // Method used is inspired by the PWM library of Mark Cooke: https://github.com/micooke/PWM
  // Note: Timer2 has more prescale values than Timer0/Timer1
  //unsigned long ulCnt=F_CPU/ulFrequency;
  uint8_t nPrescaler=1;
    // TODO: fix needed for ATtiny85
#if defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny25__)
  // smaller code for tiny85 (actually no real difference when using an optimizer)
  while(nPrescaler<(sizeof(aPrescale1)/sizeof(uint16_t)-1) && (F_CPU/ulFrequency/aPrescale1[nPrescaler] > 255L))      // Timer1 is 8-bit on ATtiny85
    nPrescaler++;
#elif defined(__AVR_ATtiny13__)
  // smaller code for tiny13 (actually no real difference when using an optimizer)
  while(nPrescaler<(sizeof(aPrescale1)/sizeof(uint16_t)-1) && (F_CPU/ulFrequency/aPrescale1[nPrescaler] > 255L))      // timer is 8-bit on ATtiny13A
    nPrescaler++;
#else
  switch(nTimer)
  {
  case 0:
  case 1:
  case 3:     // LGT8F328P has an additional Timer3 which is similar to Timer1 
    while(nPrescaler<(sizeof(aPrescale1)/sizeof(uint16_t)-1) && (F_CPU/ulFrequency/aPrescale1[nPrescaler] > (nTimer==0? 255L : 65535L)))      // timer 1 is 16-bit with 64K levels
      nPrescaler++;
    break;
  case 2:
    while(nPrescaler<(sizeof(aPrescale2)/sizeof(uint16_t)-1) && (F_CPU/ulFrequency/aPrescale2[nPrescaler] > 255L))      // timer 2 is 8-bit, but has more prescaler options
      nPrescaler++;
    break;
  }
#endif
  return(nPrescaler);
} 

int FastPwmPin::enablePwmPin(const int nPreferredPin, unsigned long ulFrequency, uint8_t nPeriodPercentage)   //  nPreferredPin=0
{ // Enable FastPwm on the desired pin
  // Note: since not all MCU's support fast PWM on every pin, the nPreferredPin indicates preference
  // Depending on the MCU a different pin may actually be enabled, pin number is returned
  // Supported frequencies:
  // LGT8F328P@32MHz, : 1Hz - 8.00MHz (8000000L)
  // ATmega328@16MHz, ATmega168@8MHz: 1Hz - 4.00MHz (4000000L)
  // The period (duty-cycle) percentage 1-99%. When larger than 50% the output mode is inverting. The resolution is lower at higher frequencies.
  
#if defined (ARDUINO_ARCH_ESP8266)
  if(nFrequency!=0)
    analogWriteFreq(ulFrequency); // Note: analogWriteFreq(0);  gives a spontaneous WDT reset
  analogWrite(nPreferredPin, 1024/(100/nPeriodPercentage);  // default range is 1024, use 0 to start quiet using pulse-width zero
  return(nPreferredPin);
#elif defined(__AVR_ATmega168P__)  ||  defined (__AVR_ATmega168__) || defined (__AVR_ATmega328P__) ||  defined (__AVR_ATmega328__)
  //
  //      ATmega328/168
  //

  // ATtiny168@8MHz/3v3 can generate 31.25kHz - 4.0MHz fast PWM
  // ATmega328/168 (Uno/Nano/Pro Mini): pin D3 or pin D11 (D11 toggle only)

  // https://www.arduino.cc/reference/en/language/functions/analog-io/analogwrite/
  // PWM pins ATmega168P: 3, 5, 6, 9, 10, and 11.
  // Timer2: D3, D11 
  // Timer1: D9, D10
  // Timer0: D5, D6

  // Timer3: LGT8F328P D1, D2

  // LGT8F328P PWM3 on pins 1,2 (QFP32)
  // NOTE: For proper support of the LGT8F328P SSOP20 use this PR: https://github.com/LaZsolt/lgt8fx
  // The 328D-SSOP20 defined in dbuezas/lgt8fx has no proper definition of OCR3A

  if((nPreferredPin!=11 && nPreferredPin!=3 && nPreferredPin!=9 && nPreferredPin!=10) || nPeriodPercentage>99 || nPeriodPercentage==0)
#if defined(OCR3A)      // LGT8F328P PWM3 on pins 1,2
    if(nPreferredPin!=1 && nPreferredPin!=2)
#endif
    return(-1);

  // Pin D3: Timer2, OC2B
  // Pin D11: Timer2, OC2A
  // ATmega168 @ 8MHz:
  // pwm-pulse OCR2B=0x00, OCR2A=0xFF => f=31.25kHz
  // pwm OCR2B=0x7F, OCR2A=0xFF => f=31.25kHz
  // pwm OCR2B=0x07, OCR2A=0x0F => f=499kHz
  // pwm OCR2B=0x03, OCR2A=0x07 => f=999kHz
  // pwm OCR2B=0x02, OCR2A=0x03 => f=1.99MHz (25%)
  // pwm OCR2B=0x01, OCR2A=0x03 => f=2.00MHz (50%)
  // pwm OCR2B=0x01, OCR2A=0x02 => f=2.66MHz (22%-33%)
  // pwm OCR2B=0x00, OCR2A=0x01 => f=4.00MHz (50%)

  byte nPrescale=1;  // 1=CK/1 (no prescaling)
  if(nPreferredPin==3 || nPreferredPin==11)
  { // TIMER0, TIMER2 have 8-bit resolution
    nPrescale=findPrescaler(ulFrequency, 2);
    if(nPreferredPin==3)
      TCCR2A = 1<<COM2B1 | (nPeriodPercentage>50)<<COM2B0 | 1<<WGM21 | 1<<WGM20;    // Clear/Set (non-inverting), fast mode 7
    else
    {
      // See https://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM
      // On 168 pin D11 only toggle mode works, at half the frequency: COM2A0 | WGM22, 
      // inverting/non-inverting modes seem not working on pin 11 in fast mode!
      // See also https://electronics.stackexchange.com/questions/49401/cant-set-to-fast-pwm-ocra-mode
      // (proposed solution won't work)
      ulFrequency*=2;   // compensate for half frequency in toggle mode
      TCCR2A = _BV(COM2A0) | _BV(WGM21) | _BV(WGM20);   // toggle mode, fast mode 7
    }
    TCCR2B = 1<<WGM22 | nPrescale<<CS20;  // fast mode 7, div/1
    // F_CPU/ulFrequency is only valid when result < 255 (OCR2A is 8-bit value)
    // On LGT8F328P F_CPU can be 32M, resulting in earlier overflow (at around 128 KHz)
    OCR2A = (F_CPU/(ulFrequency*aPrescale2[nPrescale]))-1; // pwm top, F_CPU/freq -1
    OCR2B = (OCR2A+1)/(100.0/(nPeriodPercentage>50?(100-nPeriodPercentage):nPeriodPercentage))-1; // pwm bottom, determines duty cycle, for 50%: (top+1)/2-1; thanks @cesarab for more accuracy!
  }
  
  else
#if defined(OCR3A)      // LGT8F328P PWM3 on pins 1,2 requires definitions in updated core
  //#define OCR3A _SFR_MEM16(0x98)
  // support for 16-bit Timer3 on LGT8F328P (#1=PF1/OC3A=D1, #2=PF2/OC3B=D2,
  if(nPreferredPin==2 || nPreferredPin==1)
  { // LGT8F328P PWM3 on pins 1,2 using 16-bit Timer3, same as Timer1
    // Pin 2 is supported on models QFP32 and SSOP20, pin 1 only on QFP32. QFP48 was not tested (yet).
    nPrescale=findPrescaler(ulFrequency, 3);
    if(nPreferredPin==1)
    {
      // Like AVR, LGT8F328P only supports toggle-mode on OC3A
      ulFrequency*=2;   // compensate for half frequency in toggle mode
      TCCR3A = 3<<WGM30 | 1<<COM3A0;    // toggle mode, fast mode 15

      // NOTE: On model QFP32 using OC3A on pin 1 disables using Serial TX on pin 1! Alternative Serial2 seems not supported (yet).
      // On model FPQ32, OC3A is on PF!, via shared pin PD1 (D1/TX). To enable it, PD1 needs to be disabled (set as input) and PF1 as output
      // On models SSOP20, OC3A is on PF1, via shared pin #1 (PC6/RST). To enable it, PC6 needs to be disabled (set as input) and PF1 as output
      // For now OC3A/pin 1 is only supported on model QFP32
      // WARNING: using TX for high frequency PWM may hamper recognition of the Holtec USB chip and brick your board! 
      // When my board failed to flash, I used a small cap between pin 1 and 3V3 to block the high-freq. signal and succefully flashed the board.
      DDRD&= ~_BV(1);
      DDRF|=_BV(1);
    }
    else
    {
      TCCR3A = (3<<WGM30) | (1 << COM3B1)| ((nPeriodPercentage>50)<<COM3B0);    // Fast PWM mode 15, Clear OC3B on Compare Match, set OC3B at BOTTOM (non-inverting for Period<=50%)

      // On models SSOP20 and QFP32, OC3B is on PF2, via shared pin D2 (PD2). To enable it, PD2 needs to be disabled (set as input) and PF2 as output
      DDRD&= ~_BV(2);   // pinMode(2, INPUT);
      DDRF|=_BV(2);
    }
    TCCR3B  = (1<<WGM33) | (1<<WGM32) | (nPrescale<<CS30); //  fast mode 15 (TOP in OCR3A) , nPrescale divider
    OCR3A = (F_CPU/(ulFrequency*aPrescale1[nPrescale]))-1; // pwm top, F_CPU/freq -1 // pwm top, used as BOTTOM for OC3A (D1) in WGM mode 3
    OCR3B = (OCR3A+1)/(100.0/(nPeriodPercentage>50?(100-nPeriodPercentage):nPeriodPercentage))-1; // pwm bottom, determines duty cycle, for 50%: (top+1)/2-1 (should be below top in OCR3A)
      
    return(nPreferredPin);  // return now, to avoid resetting pin to output.
  }
  else
#endif    
  { // pin 9 or 10, 16-bit Timer1, almost identical to Timer1 on ATtiny44A and ATmega8A
    nPrescale=findPrescaler(ulFrequency, 1);
    if(nPreferredPin==9)
    {
      ulFrequency*=2;   // compensate for half frequency in toggle mode
      TCCR1A = 3<<WGM10 | 1<<COM1A0;    // toggle mode, fast mode 15
    }
    else
      TCCR1A = 3<<WGM10 | (1 << COM1B1)| ((nPeriodPercentage>50)<<COM1B0);    // Fast PWM mode 15, Clear OC0B on Compare Match, set OC0B at BOTTOM (non-inverting for Period<=50%)
    TCCR1B  = (1<<WGM13) | (1<<WGM12) | (nPrescale<<CS10); //  fast mode 15 (TOP in OCR1A) , nPrescale divider
    // Calculate top according datasheet formula: Fpwm = Fsys/(Prescale*(1+TOP)) => TOP=Fsys/(Fpwm*Prescale) - 1
    OCR1A = (F_CPU/(ulFrequency*aPrescale1[nPrescale]))-1; // pwm top, F_CPU/freq -1 // pwm top, used as BOTTOM for OC0A (D0) in WGM mode 3
    OCR1B = (OCR1A+1)/(100.0/(nPeriodPercentage>50?(100-nPeriodPercentage):nPeriodPercentage))-1; // pwm bottom for pin D1, determines duty cycle, for 50%: (top+1)/2-1 (should be below top in OCR0A)
  }
  pinMode(nPreferredPin,OUTPUT);          // Set pin to output
  return(nPreferredPin);
#elif defined (__AVR_ATmega8__)
  //
  //      ATmega8/ATmega8A
  //
  // ATmega8 / ATmega8A has two timers with output pins: PB1(D9)=OC1A, PB2(D10)=OC1B, PB3(D11)=OC2, 
  // Timers: TC0:8-bit without PWM, TC1:16-bit with PWM, TC2:8-bit with PWM
  // Prescaler for Timer2 is same as ATmega328/168 but the control register is different. Timer2 only supports 50% toggle on pin 11.
  //
  // Timer2 is quite limited. Fast PWM mode for Timer2 only supports 7 fixed frequencies. Therefor we use (toggle only) CTC mode to allow 256 frequencies without prescaler. 
  // By activating the prescaler for frequencies below 40kHz we can reach lower frequencies.
  // Measured frequencies on ATmega8A @ 8Mzh using pin 11 (Timer2): 16Hz-4Mhz (toggle only)
  //
  // Timer1: 16 bit resolution. Note: seems also used by Serial in miniCore
  // Pin 9 appears to only support toggle mode
  // Measured frequencies on ATmega8A @ 8Mzh using pin 9 (Timer1): 1Hz-4Mhz (toggle only)
  // Measured frequencies on ATmega8A @ 8Mzh using pin 10 (Timer1): 1Hz-4Mhz (PWM resolution depends on frequency)
  //
  if((nPreferredPin!=11 && nPreferredPin!=10  && nPreferredPin!=9) || nPeriodPercentage>99 || nPeriodPercentage==0)
    return(-1);

  byte nPrescale=1;  // 1=CK/1 (no prescaling)
  if(nPreferredPin==11)
  { 
    nPrescale=findPrescaler(ulFrequency, 2);
    TCCR2 = 0<<COM21 | 1<<COM20 | 1<<WGM21 | 0<<WGM20 | nPrescale<<CS20;    // toggle mode COM21:0=1, CTC mode WGM21:0=2
    ulFrequency*=2;   // compensate for half frequency in toggle mode
    OCR2 = (F_CPU/(ulFrequency*aPrescale2[nPrescale]))-1; // pwm top, F_CPU/freq -1, CTC mode freq is F_CPU /2 when OCR2=0
  }
  else
  { // 16-bit Timer1, almost identical to Timer1 on ATtiny44A
    nPrescale=findPrescaler(ulFrequency, 1);
    if(nPreferredPin==9)
    {
      ulFrequency*=2;   // compensate for half frequency in toggle mode
      TCCR1A = 3<<WGM10 | 1<<COM1A0;    // toggle mode, fast mode 15
    }
    else
      TCCR1A = 3<<WGM10 | (1 << COM1B1)| ((nPeriodPercentage>50)<<COM1B0);    // Fast PWM mode 15, Clear OC0B on Compare Match, set OC0B at BOTTOM (non-inverting for Period<=50%)
    TCCR1B  = (1<<WGM13) | (1<<WGM12) | (nPrescale<<CS10); //  fast mode 15 (TOP in OCR1A) , nPrescale divider
    OCR1A = (F_CPU/(ulFrequency*aPrescale1[nPrescale]))-1; // pwm top, F_CPU/freq -1 // pwm top, used as BOTTOM for OC1A (D0) in WGM mode 3
    OCR1B = (OCR1A+1)/(100.0/(nPeriodPercentage>50?(100-nPeriodPercentage):nPeriodPercentage))-1; // pwm bottom, determines duty cycle, for 50%: (top+1)/2-1 (should be below top in OCR1A)
  }
  pinMode(nPreferredPin,OUTPUT);          // Set pin to output
  return(nPreferredPin);
#elif defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny25__)
  //
  //      ATtiny85
  //
  // ATtiny85@1MHz can generate 4.35kHz - 16.16MHz fast PWM (Fast PLL for >250 kHz)
  // For frequencies below 4kHz the prescaler is set using CS00:CS02 => 1Hz - 16MHz can be generated
  // ATtiny85: pin D1, D4 (D0/D3 only as inverted for D1/D4, 51%-99%)
  if((nPreferredPin!=4 && nPreferredPin!=3 && nPreferredPin!=1 && nPreferredPin!=0)  || nPeriodPercentage>99 || nPeriodPercentage==0)
    return(-1);
  // http://www.technoblogy.com/show?QVN - waveform generation
  // http://www.technoblogy.com/show?LE0 - four PWMs on ATtiny85
  // https://www.re-innovation.co.uk/docs/fast-pwm-on-attiny85/
  
  // Tested with
  // ATtiny85 @ 16MHz (PLL) - ATTinyCore 1.33 - Option Timer 1 Clock: 64MHz/32MHz/CPU. Pin 3: 4Hz - 4MHz.
  // ATtiny85 @ 8MHz (Internal) - ATTinyCore 1.33 - Option Timer 1 Clock: 64MHz/32MHz/CPU. Pin 3: 2Hz - 4MHz
  // ATtiny85 @ 1MHz (Internal) - ATTinyCore 1.33 - Option Timer 1 Clock: 64MHz/32MHz/CPU. Pin 3: 1Hz - 4MHz. Pin 4:  1Hz - 4MHz, 50% at F<250KHz 25%PWM<50% 
  byte nPrescale=findPrescaler(ulFrequency, 1);   // ATtiny85 has 16 prescalers, 1=CK/1 (no prescaling), 15=CK/16384

  if(nPreferredPin==4 || nPreferredPin==3)
  {
    TCCR1 = 0<<PWM1A | 0<<COM1A0 | nPrescale<<CS10;
    GTCCR = 1<<PWM1B | 1<<COM1B1 | (nPeriodPercentage>50)<<COM1B0;      // PWM on B, clear/set (non-inverting) or set-clear (inverting) mode
    // http://www.technoblogy.com/show?LE0 :
    // "There's a bug in current versions of the ATtiny85, and to use inverted mode on OC1B you also have to set COM1A0 to a non-zero value"
    // see also https://electronics.stackexchange.com/questions/97596/attiny85-pwm-why-does-com1a0-need-to-be-set-before-pwm-b-will-work
    // The ATtiny85 datasheet in the Errata (section 27.2.3#4 / page 213) says this about ATtiny45:
    // "Timer Counter1 PWM output OC1B-XOC1B does not work correctly. Only in the case when the control bits, COM1B1 and COM1B0 are in the
    // same mode as COM1A1 and COM1A0, respectively, the OC1B-XOC1B output works correctly."
    if(nPeriodPercentage>50)
      TCCR1 |=  3<<COM1A0;    // fix for bug also present in ATtiny85
    if(nPreferredPin==3) // PWM top/bottom doesn't work on pin D3, only on 4
      GTCCR = 1<<PWM1B | 0<<COM1B1 | 1<<COM1B0;     // PWM on B, inverted mode only! (i.e. 90% pwm works okay, 10% is reversed to 90%)
  }
  else
  {
    // http://www.technoblogy.com/show?LE0 :
    // "There's a bug in current versions of the ATtiny85, and to use inverted mode on OC1B you also have to set COM1A0 to a non-zero value"
    TCCR1 = 1<<PWM1A | 1<<COM1A1  | (nPeriodPercentage>50)<<COM1A0 | nPrescale<<CS10;     // PWM on A, clear/set (non-inverting) or set-clear (inverting) mode
    GTCCR = 0<<PWM1B | 0<<COM1B0;
    if(nPreferredPin==0) // PWM top/bottom doesn't work on pin D0, only on 1
      TCCR1 = 1<<PWM1A | 0<<COM1A1  | 1<<COM1A0 | nPrescale<<CS10;      // PWM on A, inverted mode only! 
  }
/*
  // Using the fast 64MHz PLL clock, the ATtiny85 can generate up to 16Mhz clock signal, even on an 1MHz system clock
  // Above 250kHz the fast PLL clock is used, below 250kHz, the oscillator is used (@F_CPU).
  // On 1MHz ATtiny85:
  // pwm OCR0B=0x7F, OCR0C=0xFF => f=252kHz  / s=3.94KHz
  // pwm OCR0B=0x3F, OCR0C=0x7F => f=504kHz
  // pwm OCR0B=0x1F, OCR0C=0x3F => f=1.01MHz
  // pwm OCR0B=0x0F, OCR0C=0x1F => f=2.02MHz
  // pwm OCR0B=0x07, OCR0C=0x0F => f=4.04MHz (@33%-50%)
  // pwm OCR0B=0x03, OCR0C=0x07 => f=8.08MHz (@33%)
  // pwm OCR0B=0x01, OCR0C=0x03 => f=16.17MHz (@??) / s=252kHz (@33%)
  // pwm OCR0B=0x02, OCR0C=0x03 => f=16.17MHz (@??) / s=252kHz (@50%)
  // pwm OCR0B=0x01, OCR0C=0x02 => f=21.55MHz (multimeter reading)
  // pwm OCR0B=0x00, OCR0C=0x01 => NO PWM!
  OCR1B = 0x07;
  OCR1C = 0x0F; // pwm top, determines duty cycle (should be below top in OCR1B)  0x02
*/
  if(ulFrequency>=250000L && nPrescale==1)
  {
    #define FASTPWMPIN_TINY85PLL 64000000L
    OCR1C = (FASTPWMPIN_TINY85PLL/ulFrequency)-1; // pwm top, F_CPU/freq -1 // pwm top

    //  PLLCSR= 1<<PCKE | 1<<PLLE;    // enable ATTiny85 64MHz clock (high speed mode, 64MHz)
    PLLCSR= 1<<PCKE | 1<<PLLE | 1<<PLOCK;    // enable ATTiny85 64MHz clock (high speed mode+lock, 64MHz)
    //PLLCSR= 1<<LSM | 1<<PCKE | 1<<PLLE;    // enable ATTiny85 64MHz clock (low speed mode, 32MHz for VCC<2.7V)
  }
  else
  {
    //#define FASTPWMPIN_TINY85OSC 8000000L
    OCR1C = (F_CPU/(ulFrequency*aPrescale1[nPrescale]))-1; // pwm top, F_CPU/freq -1
    PLLCSR= 0;    // disable ATTiny85 64MHz clock
  }
  if(nPreferredPin==4 || nPreferredPin==3) OCR1B = (OCR1C+1)/(100.0/(nPeriodPercentage>50?(100-nPeriodPercentage):nPeriodPercentage))-1; // pwm bottom for pin D4, determines duty cycle, for 50%: (top+1)/2-1 (should be below top in OCR1C)
  if(nPreferredPin==1 || nPreferredPin==0) OCR1A = (OCR1C+1)/(100.0/(nPeriodPercentage>50?(100-nPeriodPercentage):nPeriodPercentage))-1; // pwm bottom for pin D1, determines duty cycle, for 50%: (top+1)/2-1 (should be below top in OCR1C)
  pinMode(nPreferredPin,OUTPUT);          // Set pin to output
  return(nPreferredPin);
#elif defined(__AVR_ATtiny13__)
  //
  //      ATtiny13
  //
  // ATtiny13@9.6MHz/3v3 can generate 39.5kHz - 1.6MHz fast PWM (higher becomes unstable, 5V not tested ok)
  // The prescaler is set using CS00:CS02 for lower frequencies
  if((nPreferredPin!=0 && nPreferredPin!=1) || nPeriodPercentage>99 || nPeriodPercentage==0)
    return(-1);

  byte nPrescale=1;  // 1=CK/1 (no prescaling)
  nPrescale=findPrescaler(ulFrequency, 0);
  if(nPreferredPin==0)  // TODO: only pin==1 seems to work ok
  {
    //TCCR0A = ((1<<WGM00) | (1<<WGM01) | (1 << COM0A1)| ((nPeriodPercentage>50) << COM0A0)) ; // Fast PWM mode 7, Clear OC0A on Compare Match, set OC0A at TOP (non inverting)
    // inverting/non-inverting modes seem not working on pin 0 in fast mode!
    // See also https://electronics.stackexchange.com/questions/49401/cant-set-to-fast-pwm-ocra-mode
    ulFrequency*=2;   // compensate for half frequency in toggle mode
    TCCR0A = 3<<WGM00 | 1<<COM0A0;    // toggle mode, fast mode 7
  }
  else
    TCCR0A = ((1<<WGM00) | (1<<WGM01) | (1 << COM0B1)| ((nPeriodPercentage>50) << COM0B0)) ; // Fast PWM mode 7, Clear OC0B on Compare Match, set OC0B at TOP (non inverting)
  TCCR0B  = ((1<<WGM02) | (nPrescale<<CS00)); //  fast mode 7, nPrescale divider
  
  // pwm OCR0A=0x80, OCR0B=0x7F => f=73.5KHz
  // pwm OCR0A=0x0F, OCR0B=0x05 => f=598khz (+/-1kHz)
  // pwm OCR0A=0x0D, OCR0B=0x05 => f=686kHz
  // pwm OCR0A=0x0A, OCR0B=0x05 => f=877kHz
  // pwm OCR0A=0x0A, OCR0B=0x08 => f=877.0kHz (+0.6kHz)
  // pwm OCR0A=0x09, OCR0B=0x04 => f=967kHz (+/-2kHz)
  // pwm OCR0A=0x08, OCR0B=0x01 => f=1.074MHz (+/-3kHz)
  // pwm OCR0A=0x06, OCR0B=0x00 => f=1.395MHz (+/-2kHz)
  // pwm OCR0A=0x05, OCR0B=0x04 => f=1.635MHz (+/-3kHz)
  // pwm OCR0A=0x05, OCR0B=0x00 => f=1.635MHz
  // pwm OCR0A=0x04, OCR0B=0x00 => f=1.975MHz (+/-2kHz)
  // pwm OCR0A=0x04, OCR0B=0x03 => f=1.979MHz (+/-2kHz)
  // pwm OCR0A=0x03, OCR0B=0x02 => f=2.50MHz (+/-7kHz)
  // pwm OCR0A=0x02, OCR0B=0x01 => f=3.39MHz
  // pwm OCR0A=0x02, OCR0B=0x00 => f=3.37MHz (+/-10kHz)
  // pwm OCR0A=0x01, OCR0B=0x00 => f=5.10MHz (+/-7kHz)
  OCR0A = (F_CPU/(ulFrequency*aPrescale1[nPrescale]))-1; // pwm top,  used as BOTTOM for OC0A (D0) in WGM mode 3, F_CPU/freq -1
  OCR0B = (OCR0A+1)/(100.0/(nPeriodPercentage>50?(100-nPeriodPercentage):nPeriodPercentage))-1; // pwm bottom for pin D1, determines duty cycle, for 50%: (top+1)/2-1 (should be below top in OCR1A)
  DDRB |= (1 << nPreferredPin); // pinMode(nPreferredPin,OUTPUT);          // Set pin to output
  return(nPreferredPin);
#elif (defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__))
  //
  //      ATtiny24/24A/44/44A/84/84A
  //
  //  ATtiny24A/44A/84A PWM pins: PB2(D8)=OC0A, PA7(D7)=OC0B, PA6(D6)=OC1A, PA5(D5)=OC1B
  //  TC0: 8-bit. Note that Timer0 is often used for delay().
  //  TC1: supports 16-bit precision on PA6/D6 (OC1A) and PA5/D5 (OC1B)
  // Tested frequencies @8MHz on pin 5: 1 Hz - 4 MHz (+10%)
  // Tested frequencies @8MHz on pin 6: 1 Hz - 4 MHz (+10%)
  // Tested frequencies @8MHz on pin 7: 32 Hz - 4 MHz (+10%)
  // Tested frequencies @8MHz on pin 8: 16 Hz - 4 MHz (+10%)
  // Pins 6 and 8 support toggle only
  if(nPreferredPin<5 || nPreferredPin>8  || nPeriodPercentage>99 || nPeriodPercentage==0)
    return(-1);
  byte nPrescale=1;  // 1=CK/1 (no prescaling)
  if(nPreferredPin==8 || nPreferredPin==7)
  { // 8-bit Timer0
    nPrescale=findPrescaler(ulFrequency, 0);
    if(nPreferredPin==8)
    {
      // Like the ATtiny13A, only toggle mode seems to work on OC0A (PB2/D8)
      // See also https://electronics.stackexchange.com/questions/49401/cant-set-to-fast-pwm-ocra-mode
      ulFrequency*=2;   // compensate for half frequency in toggle mode
      TCCR0A = 3<<WGM00 | 1<<COM0A0;    // toggle mode, fast mode 7
    }
    else
      TCCR0A = 3<<WGM00 | (1 << COM0B1)| ((nPeriodPercentage>50)<<COM0B0);    // Fast PWM mode 7, Clear OC0B on Compare Match, set OC0B at BOTTOM (non-inverting for Period<=50%)
    TCCR0B  = (1<<WGM02) | (nPrescale<<CS00); //  fast mode 7, nPrescale divider
    OCR0A = (F_CPU/(ulFrequency*aPrescale1[nPrescale]))-1; // pwm top, F_CPU/freq -1
    OCR0B = (OCR0A+1)/(100.0/(nPeriodPercentage>50?(100-nPeriodPercentage):nPeriodPercentage))-1; // pwm bottom for pin D1, determines duty cycle, for 50%: (top+1)/2-1 (should be below top in OCR0A)
  }
  else
  { // 16-bit Timer1
    nPrescale=findPrescaler(ulFrequency, 1);
    if(nPreferredPin==6)
    {
      ulFrequency*=2;   // compensate for half frequency in toggle mode
      TCCR1A = 3<<WGM10 | 1<<COM1A0;    // toggle mode, fast mode 15
    }
    else
      TCCR1A = 3<<WGM10 | (1 << COM1B1)| ((nPeriodPercentage>50)<<COM1B0);    // Fast PWM mode 15, Clear OC0B on Compare Match, set OC0B at BOTTOM (non-inverting for Period<=50%)
    TCCR1B  = (1<<WGM13) | (1<<WGM12) | (nPrescale<<CS10); //  fast mode 15 (TOP in OCR1A) , nPrescale divider
    OCR1A = (F_CPU/(ulFrequency*aPrescale1[nPrescale]))-1; // pwm top, F_CPU/freq -1 // pwm top, used as BOTTOM for OC1A (D0) in WGM mode 3
    OCR1B = (OCR1A+1)/(100.0/(nPeriodPercentage>50?(100-nPeriodPercentage):nPeriodPercentage))-1; // pwm bottom, determines duty cycle, for 50%: (top+1)/2-1 (should be below top in OCR1A)
  }
  pinMode(nPreferredPin, OUTPUT);
  return(nPreferredPin);
#else
#error Unsupported MCU for FastPwmPin
  //
  //      Unknown!
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


#if defined(TINYX4_ENABLE_WDTMILLIS)
//#error test
// Using FastPwmPin on ATtiny44A with pins 7 or 8 uses Timer0, which impacts delay() 
// Therefore we redefine millis() and delay() to use the watchdog timer instead.
// This code is based on an older version of the MicroCore ATtiny13A implementation of millis()
// See https://github.com/MCUdude/MicroCore/blob/master/avr/cores/microcore/wiring.c
// For used version see https://github.com/MCUdude/MicroCore/blob/5a4caa5e4b783f891ec8c8f1a25a8aac6e2324e0/avr/cores/microcore/wiring.c
#include <avr/wdt.h>

// The millis counter is based on the watchdog timer, and takes very little processing time and power.
// If 16 ms accuracy is enough, I strongly recommend you to use millis() instead of micros().
volatile uint32_t wdt_interrupt_counter = 0;

// This ISR will execute every 16 ms, and increase 
ISR(WDT_vect)
{
  wdt_interrupt_counter++;
}

bool _fWdtInit=false;
void wdt_init()
{
/*
  if(_fWdtInit)
    return;
*/
  _fWdtInit=true;

  // Enable WDT interrupt and enable global interrupts  
  cli();  // Disable global interrupts      
  wdt_reset();  // Reset watchdog
  WDTCSR = _BV(WDIE);  // Set up WDT interrupt with 16 ms prescaler (Note: different names than ATtiny13A)
  sei();  // Enable global interrupts
}

// Since the WDT counter counts every 16th ms, we'll need to multiply to get the correct millis value.
// The WDT uses it's own clock, so this function is valid for all F_CPUs.
// Note: According the ATtiny44A datasheet the Watchdog oscillator frequency is about 118 kHz (at 5V/25`C)
// According the ATtiny13A datasheet that Watchdog oscillator frequency is about 111 kHz (at 5V/25`C)
// Using prescaler WDP0[0:3] of zero the WDT times out at 2K cycles or 16ms (for both ATtiny13A and ATtiny44).
// Therefor a multiplier of 16 was used in Micro Core for the ATtiny13A. For ATtiny44A 19 was measured to be more accurate.
uint32_t wdt_millis()
{
  if(!_fWdtInit)
    wdt_init();
  return wdt_interrupt_counter * 19;
}

void wdt_delay(uint16_t ms) 
{
  unsigned long msstart=wdt_millis();
  while(wdt_millis()<msstart+ms);
} 
#endif // TINYX4_ENABLE_WDTMILLIS


