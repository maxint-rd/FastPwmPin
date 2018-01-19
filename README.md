# FastPwmPin
Arduino library to generate a fast PWM signal on an output pin at maximum frequency. Example included.

### Introduction
FastPwmPin provides a means to generate a high frequency PWM signal on one specific output pin. Where the regular Arduino analogWrite() function allows for generating a low frequency signal, this library achieves high frequencies using fast timer manipulation. The library produces a PWM signal on a single output pin. The frequency and period (duty cycle) can be selected. The library supports multiple MCU's. The capabilities depend on the specific MCU.

### Support for different MCUs
This library supports generating a high frequency signal on different MCUs such ATmega 328, 168 and ATtiny85. Depending on the MCU, it uses different timers and registers to produce the high frequency signal. The table below gives an overview:

MCU (Board) | Available pins | Timer used | Remarks
------------ | ------------- | ------------- | -------------
ATmega328 ATmega168<br>(Aduino Uno, Nano, Pro Mini) | 3, 11 | Timer2 | Pin 11 only supports toggle mode (50% PWM)
ATtiny85 | 1, 4 | Timer1 | Pins 0, 3 can also be used, but only inverted (51%-99%)
ATtiny13A | 0, 1 | Timer0 | Pins 0 only supports toggle mode (50% PWM)
ESP8266 |  |  | NOT SUPPORTED (YET)
ESP32 |  |  | NOT SUPPORTED (YET)
STM32 |  |  | NOT SUPPORTED (YET)

### Tested frequencies
This library has been tested on multiple MCU's under various condititions\*. The generated signal frequency has been measured using different methods. The table below lists frequencies measured\*\*:

MCU (Board) | Clock (voltage) | Highest frequency | Lowest frequency | Remarks
------------ | ------------- | ------------- | ------------- | -------------
ATmega168 (Pro Mini) | 8 Mhz (3v3) | 4.0 MHz | 31.25 kHz | toggle only on pin 11 and highes frequecies
ATtiny85 | 1Mzh/8MHz (3v3) | 16.16 MHz | 4.35 kHz | when > 500 kHz fast PLL clock is activated
ATtiny13A | 9.6MHz (3v3) | 1.6 MHz | 39.5 kHz | frequencies > 1.6 MHz are instable

\* *If you tested this library on a different board-setup, please send me your findings, so I can update the table.*<br>
\*\* *Frequency was measured using UT89C multimeter, DSO112 mini oscilloscope, Arduino [FreqCount](https://github.com/PaulStoffregen/FreqCount/tree/master/examples/Serial_Output) serial example on 16MHz Nano.*

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
   uint8_t nPeriodPercentage=FASTPWMPIN_TOGGLE
);

Parameters:
   nPreferredPin - prefered pin to generate the high frequency signal
   ulFrequency - frequency in Hertz
   nPeriodPercentage - PWM percentage

Return value:
   When succesful the method returns the preferred pin as set. If unsuccesful -1 is returned.
```

See the enclosed [example](examples/FastPwmPin) for more details.

### Features & limitations
 - For ATmega 328/168 Timer2 is used. This impacts the tone() function.
 - On the ATtiny85 Timer1 is used, which impacts regular PWM output.
 - On the ATtiny13A Timer0 is used. For some cores this impacts the delay() and millis() functions.
 - The resolution and frequency of the actually generated signal depends on the MCU used. The supplied parameters may be truncated during calculations.
 - At higher frequencies the resolution of the period (duty-cycle) gets more and more limited (converging to 50%).
 - The stability (jitter) of the generated signal depends on the MCU and the selected frequency and duty cycle. In testing the ATtiny13A showed more jitter than the ATtiny85 at higher frequencies. The jitter can easily be measured using the Arduino [FreqCount](https://github.com/PaulStoffregen/FreqCount/tree/master/examples/Serial_Output) serial example an the serial plotter of the Arduino IDE.

### Credits
- This library is based on information found in various sources. See the links below for references.

### Links
- About regular PWM:<br>
  https://www.arduino.cc/reference/en/language/functions/analog-io/analogwrite/
- About register manipulation to enable fast PWM<br>
  https://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM [original with more graphics](http://www.righto.com/2009/07/secrets-of-arduino-pwm.html)<br>
  http://www.technoblogy.com/show?LE0 - *"four PWMs on ATtiny85"*<br>
  https://www.re-innovation.co.uk/docs/fast-pwm-on-attiny85/
- About limited modes on certain pins:<br>
  https://electronics.stackexchange.com/questions/49401/cant-set-to-fast-pwm-ocra-mode

### Disclaimer
- All code on this GitHub account, including this library is provided to you on an as-is basis without guarantees and with all liability dismissed. It may be used at your own risk. Unfortunately I have no means to provide support.
