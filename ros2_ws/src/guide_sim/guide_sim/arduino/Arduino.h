// A host (PC) implementation of the parts of the Arduino core the guide car's
// firmware uses, so the real .ino compiles into a shared library and runs
// inside the hardware-in-the-loop simulator (guide_sim/firmware.py).
//
// Time is virtual: the simulator sets micros() before each loop() call, and
// delay()/delayMicroseconds()/pulseIn() advance it as they would on the board.
// Pins, the serial port and the ultrasonic echo are plain state the simulator
// reads and writes through the C functions in arduino_host.cpp.
#pragma once

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

typedef uint8_t byte;
typedef bool boolean;

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define CHANGE 1
#define FALLING 2
#define RISING 3

#ifndef NUM_PINS
#define NUM_PINS 70  // an Arduino Mega has 70 pin numbers
#endif

class HostSerial {
 public:
  void begin(unsigned long baud);
  int available();
  int read();
  size_t write(uint8_t c);
  size_t print(const char *s);
  size_t print(char c);
  size_t print(int v);
  size_t print(unsigned int v);
  size_t print(long v);
  size_t print(unsigned long v);
  size_t print(double v, int digits = 2);
  size_t println();
  template <typename T>
  size_t println(T v) {
    size_t n = print(v);
    return n + println();
  }
  operator bool() { return true; }
};
extern HostSerial Serial;

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t value);
int digitalRead(uint8_t pin);
void analogWrite(uint8_t pin, int value);
int analogRead(uint8_t pin);
unsigned long millis();
unsigned long micros();
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout = 1000000UL);
int digitalPinToInterrupt(int pin);
void attachInterrupt(int interrupt, void (*isr)(), int mode);
void detachInterrupt(int interrupt);
void noInterrupts();
void interrupts();
