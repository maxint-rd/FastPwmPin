# FastPwmPin
Arduino library to generate a fast PWM signal on an output pin at maximum frequency. Example included.

### Introduction
FastPwmPin provides a means to generate a high frequency PWM signal on one specific output pin. Where the regular Arduino analogWrite() function allows for generating a low frequency signal, this library achieves high frequencies using fast timer manipulation. The library produces a PWM signal on an output pin; the frequency and period can be selected. The library supports multiple MCU's. The capabilities depend on the specific MCU.

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
int FastPwmPin::enablePwmPin(const int nPreferredPin=0, unsigned long ulFrequency=0L, uint8_t nPeriodPercentage=FASTPWMPIN_TOGGLE);
nPreferredPin - prefered pin to generate the high frequency signal
ulFrequency - frequency in Hertz
nPeriodPercentage - PWM percentage
```

See the enclose example code for more details.

### Features & limitations
 - For ATmega 328/168 Timer2 is used. This impacts the tone() function.
 - On the ATtiny85 Timer1 is used, which impacts regular PWM output.
 - On the ATtiny85 Timer0 is used. For some cores this impacts the delay() and millis() functions.
 - The resolution and frequency of the actually generated signal depends on the MCU used. The supplied parameters may be truncated during calculations.
 - At higher frequencies the resolution of the period (duty-cycle) gets more and more limited (converging to 50%) 

### Credits
This library is based information found in various sources. See the links below for references.

### Links
- Learn more about Music Macro Language (MML) on wikipedia:<br>
   http://en.wikipedia.org/wiki/Music_Macro_Language<br>
- For downloadable MML music see http://www.archeagemmllibrary.com/<br>
- Extensive MML reference guide (not all commands supported):<br>
   http://woolyss.com/chipmusic/chipmusic-mml/ppmck_guide.php<br>
- Info about using PWM and other methods to generate sound:<br>
   https://developer.mbed.org/users/4180_1/notebook/using-a-speaker-for-audio-output/

### Disclaimer
- All code on this GitHub account, including this library is provided to you on an as-is basis without guarantees and with all liability dismissed. It may be used at your own risk. Unfortunately I have no means to provide support.
