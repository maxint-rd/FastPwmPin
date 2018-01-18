/*
  Blink with fast PWM pin
  Turns on an LED on and off, repeatedly, along with fast toggle of a specific pin.
  For more information see https://github.com/maxint-rd/FastPwmPin

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO 
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino model, check
  the Technical Specs of your board  at https://www.arduino.cc/en/Main/Products
  For ATtiny85 and ATtiny13A the library set LED_BUILTIN to pin 2.
  
  This example code is based on the Arduino Blink example, released in the public domain.
  Original version modified by Scott Fitzgerald, Arturo Guadalupi & Colby Newman.
  This version modified 17 January 2018 by Maxint R&D (github.com/maxint-rd)
*/

#include <FastPwmPin.h>

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  // Enable fast PWM on a pin
  // ATmega328/168 (Uno/Nano/Pro Mini): pin D3 or pin D11 (D11 toggle only)
  // ATtiny85: pin D1, D4 (D0/D3 only as inverted for D1/D4, 51%-99%)
  // ATtiny13A: pin D1, D0 (D0 toggle only)
  FastPwmPin::enablePwmPin(11, 4000000L, 50);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(100);                       // wait for a second
}
