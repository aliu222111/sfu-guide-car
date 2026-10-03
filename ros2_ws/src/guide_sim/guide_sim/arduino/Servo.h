// Host version of the Arduino Servo library: each attached pin's pulse width
// (microseconds) is plain state the simulator reads (host_servo_us), as an
// RC steering servo or ESC would read the 50 Hz pulse on its signal wire.
#pragma once
#include "Arduino.h"

void host_servo_write(uint8_t pin, int us);

class Servo {
 public:
  uint8_t attach(int pin) {
    pin_ = pin;
    host_servo_write(pin_, 1500);  // the library starts every servo at its centre
    return 0;
  }
  uint8_t attach(int pin, int, int) { return attach(pin); }
  void writeMicroseconds(int us) {
    if (pin_ >= 0) host_servo_write(pin_, us);
  }
  void write(int degrees) { writeMicroseconds(544 + (long)degrees * (2400 - 544) / 180); }
  bool attached() { return pin_ >= 0; }
  void detach() {
    if (pin_ >= 0) host_servo_write(pin_, 0);  // no pulses: the ESC disarms, the servo goes limp
    pin_ = -1;
  }

 private:
  int pin_ = -1;
};
