/*
  Blink a led using a PWM pin
  Turns on an LED on and off, repeatedly, using slow PWM toggle of a specific pin.

  Note: Not every supported MCU/pin can generate 1 Hz. See these test results:
  - ATmega328 @ 16 MHz, Arduino Pro Mini, output 40 Hz - 4 MHz on pin 11 (toggle only)
                                          output 80 Hz - 4 MHz on pin 3
                                          output  1 Hz - 4 MHz on pin 9 (toggle only)
                                          output  1 Hz - 4 MHz on pin 10
  - ATmega8A @ 8 MHz, MiniCore, output 1Hz - 4MHz on pin 9 using 16-bit Timer1 (prescaler for <128Hz)
  - ATtiny85 @ 1 MHz, ATTinyCore, output 1Hz - 4MHz on pin 1 or 4
  - ATtiny13A @ 9,6 Mhz, MicroCore, output 38Hz - 4MHz on pin 1 or 4
  - ATtiny44A @ 8 Mhz, ATTinyCore, output 1Hz - 4MHz on pins 5,6 using 16-bit Timer1 (prescaler for <64Hz)
                                   output 32,16Hz - 4MHz on pins 7,8 using 8-bit Timer0 (prescaler for <40kHz)

  Made by Maxint-RD MMOLE 2020
  For more information see https://github.com/maxint-rd/FastPwmPin
*/

#include <FastPwmPin.h>
#define PIN_PWM 4


void setup() {
  // enable PWM-blinking on the specified pin using low frequency PWM
  FastPwmPin::enablePwmPin(PIN_PWM, 4L, 50);
  delay(3000);
  FastPwmPin::enablePwmPin(PIN_PWM, 1L, 10);
}

void loop() {
  // put your main code here, in this example we do nothing
}