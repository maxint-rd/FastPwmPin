# FastPwmPin
Arduino library to generate a fast PWM signal on an output pin at maximum frequency. Examples included.

### Introduction
FastPwmPin provides a means to generate a high frequency PWM signal on one specific output pin. Where the regular Arduino analogWrite() function allows for generating a fixed frequency signal, this library achieves high frequencies using fast timer manipulation. The library produces a PWM signal on a single output pin. The frequency and duty cycle can be selected. The library supports multiple MCU's. The capabilities depend on the specific MCU. While originally aimed at high frequencies, the library can now also generate very low frequencies; depending on the MCU used as low as 1 Hz.

### Support for different MCUs
This library supports generating a high frequency signal on different MCUs such ATmega 328, 168 and ATtiny85. Depending on the MCU, it uses different timers and registers to produce the high frequency signal. The table below gives an overview:

MCU (Board) | Available pins | Timer used | Remarks
------------ | ------------- | ------------- | -------------
ATmega328 ATmega168<br>(Arduino Uno, Nano, Pro Mini) | 9,10/3,11 | Timer1/Timer2 | Pins 9, 10 16-bit resolution, pins 3, 11 8-bit resolution. Pin 9, 11 only support toggle mode (50% PWM)
ATmega8A | 9,10/11 | Timer1/Timer2 | Pins 9, 10 16-bit resolution, pin 11 8-bit resolution. Pins 9, 11 toggle mode only (50% PWM)
ATtiny85 | 1,3 | Timer1 | Pins 0, 4 can also be used, but only inverted (51%-99%)
ATtiny44A | 5,6/7,8 | Timer1/Timer0 | Pins 5, 6 16-bit resolution,  pins 7, 8 8-bit resolution. Pins 6 and 8 only support toggle mode (50% PWM)
ATtiny13A | 0, 1 | Timer0 | Pin 0 only supports toggle mode (50% PWM)
ESP8266 |  |  | Minimal implementation using analogWriteFreq() and analogWrite()
ESP32 |  |  | NOT SUPPORTED (YET)
STM32 |  |  | NOT SUPPORTED (YET)

### Tested frequencies
This library has been tested on multiple MCU's under various condititions\*. The generated signal frequency has been measured using different methods. The table below lists frequencies measured\*\*:

MCU (Board) | Clock (voltage) | Highest frequency | Lowest frequency | Remarks
------------ | ------------- | ------------- | ------------- | -------------
ATmega328 (Pro Mini) | 16 Mhz (3v3/5V) | 4 MHz | 40 Hz | toggle only at highest frequencies
ATmega168 (Pro Mini) | 8 Mhz (3v3) | 4 MHz |  | toggle only on pin 11 and highest frequencies
ATmega8A | 8MHz (5V) | 4 MHz | 1 Hz | best resolution on pins 9, 10
ATtiny85 | 1Mzh/8MHz (3v3) | 16.16 MHz | 1 Hz | when > 500 kHz fast PLL clock is activated
ATtiny44A | 8MHz (3v3/5V) | 4.4 MHz | 1 Hz | lowest frequency measured on pin 7 is 32 Hz, on pin 5 it is 1 Hz
ATtiny13A | 9.6MHz (3v3) | 1.6 MHz | 38 Hz | frequencies > 1.6 MHz are instable

\* *If you tested this library on a different board-setup, please send me your findings, so I can update the table.*<br>
\*\* *Frequency was measured using UT89C multimeter, DSO112 mini oscilloscope, Arduino [FreqCount](https://github.com/PaulStoffregen/FreqCount/tree/master/examples/Serial_Output) serial example on 16MHz Nano and a logic analyzer (@16MS/s).*

### Installation/Usage
The library can be downloaded from https://github.com/maxint-rd/FastPwmPin. It can be installed as an Arduino library using the Sketch|Library menu. 
Just add the zipfile library and the enclosed example should appear in the menu automatically. 

Initialisation before Setup():
```
  // include header
  #include <FastPwmPin.h>
```

Then to initialize the high frequency signal, call the enablePwmPin() method in Setup():
```
FastPwmPin::enablePwmPin(11, 4000000L, 50);
```

The enablePwmPin() method has the following syntax:
```
int FastPwmPin::enablePwmPin(
   const int nPreferredPin=0,
   unsigned long ulFrequency=0L,
   uint8_t nDutyPercentage=FASTPWMPIN_TOGGLE
);

Parameters:
   nPreferredPin - prefered pin to generate the high frequency signal
   ulFrequency - frequency in Hertz
   nDutyPercentage - PWM percentage (duty cycle)

Return value:
   When succesful the method returns the preferred pin as set. If unsuccesful -1 is returned.
```
Note: To switch a pin fully on or fully off, you must use digitalWrite(). Duty percentages of 100 and 0 result in -1 error return value.

See the enclosed [example](examples/FastPwmPin) for more details.

### Features & limitations
 - When Timer 0 is used (ATtiny44A/13A) the delay() and millis() functions can be impacted (depending on the core used).
 - When Timer1 is used (ATtiny85/44A and ATmega 328/168/8A), regular PWM output is impacted.
 - When Timer2 is used (ATmega 328/168), the tone() function is impacted.
 - The resolution and frequency of the actually generated signal depend on the timer of the MCU used. The supplied parameters are truncated during calculations. The precision of the generated signal depends on this integer truncation as well as on the stability of the MCU clock. A crystal clock is more stable than an RC oscillator or PLL.
 - The theoretical maximum frequency at full duty cycle resulution (256 levels) is clock speed divided by 256. At higher frequencies the resolution of the duty cycle gets smaller (converging to 50%). When the MCU is running at lower voltages than specified, the higher frequencies may become unstable.
 - On ATtiny44A and ATmega328/168/8A the 16-bit Timer1 is also supported, allowing for lower frequencies and for higher PWM precision (at those lower frequencies).
 - At lower frequencies the prescaler is enabled, allowing for lower frequencies and for higher PWM precision (at those lower frequencies). The frequencies that determine the different prescaler settings were determined through experimentation and are depending on the MCU used. 
 - The stability (jitter) of the generated signal depends on the MCU and the selected frequency and duty cycle. In testing the ATtiny13A showed more jitter than the ATtiny85 at higher frequencies. The jitter can easily be measured using the Arduino [FreqCount](https://github.com/PaulStoffregen/FreqCount/tree/master/examples/Serial_Output) serial example an the serial plotter of the Arduino IDE.

### Credits
- This library is based on information found in various sources. See the links below for references.

### Links
- About regular PWM:<br>
  https://www.arduino.cc/reference/en/language/functions/analog-io/analogwrite/
- Extensive post about timers and using them for PWM:<br>
  http://www.gammon.com.au/timers
- About register manipulation to enable fast PWM:<br>
  https://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM \[[original with more graphics](http://www.righto.com/2009/07/secrets-of-arduino-pwm.html)\]<br>
  http://www.technoblogy.com/show?LE0 - *"four PWMs on ATtiny85"*<br>
  https://www.re-innovation.co.uk/docs/fast-pwm-on-attiny85/
- About limited modes on certain pins:<br>
  https://electronics.stackexchange.com/questions/49401/cant-set-to-fast-pwm-ocra-mode

### Disclaimer
- All code on this GitHub account, including this library is provided to you on an as-is basis without guarantees and with all liability dismissed. It may be used at your own risk. Unfortunately I have no means to provide support.
