// Host implementation of Arduino.h plus the C interface the simulator calls.
// The firmware's .ino is compiled into the same library by firmware_main.cpp.

#include "Arduino.h"

#include <stdarg.h>
#include <stdio.h>

#include <deque>
#include <string>

namespace {
uint64_t now_us = 0;              // virtual time
uint8_t mode_[NUM_PINS];          // pinMode
uint8_t level_[NUM_PINS];         // digitalWrite output level
int pwm_[NUM_PINS];               // analogWrite duty, -1 when the pin is driven digitally
int servo_[NUM_PINS];             // Servo pulse width in microseconds, 0 = no pulses
uint8_t input_[NUM_PINS];         // levels the simulator drives onto input pins
void (*isr_[NUM_PINS])() = {};
std::deque<uint8_t> rx_;           // laptop -> board
std::string tx_;                   // board -> laptop
uint32_t echo_us_ = 0;             // the ultrasonic echo pulse length, 0 = no pulse (dead sensor)
bool interrupts_on_ = true;

bool valid(uint8_t pin) { return pin < NUM_PINS; }
}  // namespace

HostSerial Serial;

void HostSerial::begin(unsigned long) {}
int HostSerial::available() { return (int)rx_.size(); }
int HostSerial::read() {
  if (rx_.empty()) return -1;
  int c = rx_.front();
  rx_.pop_front();
  return c;
}
size_t HostSerial::write(uint8_t c) {
  tx_.push_back((char)c);
  return 1;
}
size_t HostSerial::print(const char *s) {
  tx_ += s;
  return strlen(s);
}
size_t HostSerial::print(char c) { return write((uint8_t)c); }
static size_t print_fmt(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static size_t print_fmt(const char *fmt, ...) {
  char buf[64];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  tx_ += buf;
  return n > 0 ? (size_t)n : 0;
}
size_t HostSerial::print(int v) { return print_fmt("%d", v); }
size_t HostSerial::print(unsigned int v) { return print_fmt("%u", v); }
size_t HostSerial::print(long v) { return print_fmt("%ld", v); }
size_t HostSerial::print(unsigned long v) { return print_fmt("%lu", v); }
size_t HostSerial::print(double v, int digits) { return print_fmt("%.*f", digits, v); }
size_t HostSerial::println() { return print("\r\n"); }  // Arduino ends lines with CR LF

void pinMode(uint8_t pin, uint8_t mode) {
  if (!valid(pin)) return;
  mode_[pin] = mode;
  if (mode == INPUT_PULLUP && input_[pin] == 0xFF) input_[pin] = HIGH;
}
void digitalWrite(uint8_t pin, uint8_t value) {
  if (!valid(pin)) return;
  level_[pin] = value ? HIGH : LOW;
  pwm_[pin] = -1;
}
int digitalRead(uint8_t pin) {
  if (!valid(pin)) return LOW;
  if (mode_[pin] == OUTPUT) return level_[pin];
  if (input_[pin] == 0xFF) return mode_[pin] == INPUT_PULLUP ? HIGH : LOW;
  return input_[pin];
}
void analogWrite(uint8_t pin, int value) {
  if (!valid(pin)) return;
  if (value < 0) value = 0;
  if (value > 255) value = 255;
  pwm_[pin] = value;
  level_[pin] = value > 0 ? HIGH : LOW;
}
int analogRead(uint8_t) { return 0; }
unsigned long millis() { return (unsigned long)(now_us / 1000); }
unsigned long micros() { return (unsigned long)now_us; }
void delay(unsigned long ms) { now_us += (uint64_t)ms * 1000; }
void delayMicroseconds(unsigned int us) { now_us += us; }
unsigned long pulseIn(uint8_t, uint8_t, unsigned long timeout) {
  // The board waits about 0.5 ms for the echo pulse to start, then for it to
  // end. No pulse (echo 0: a dead sensor) or one longer than the timeout costs
  // the whole timeout and returns 0, as on the board.
  if (echo_us_ == 0 || echo_us_ + 500 > timeout) {
    now_us += timeout;
    return 0;
  }
  now_us += echo_us_ + 500;
  return echo_us_;
}
int digitalPinToInterrupt(int pin) { return pin; }
void attachInterrupt(int interrupt, void (*isr)(), int) {
  if (interrupt >= 0 && interrupt < NUM_PINS) isr_[interrupt] = isr;
}
void detachInterrupt(int interrupt) {
  if (interrupt >= 0 && interrupt < NUM_PINS) isr_[interrupt] = nullptr;
}
void noInterrupts() { interrupts_on_ = false; }
void interrupts() { interrupts_on_ = true; }

void host_servo_write(uint8_t pin, int us) {
  if (valid(pin)) servo_[pin] = us;
}

// Provided by the firmware (.ino).
void setup();
void loop();

extern "C" {
void host_reset() {
  now_us = 0;
  memset(mode_, 0, sizeof mode_);
  memset(level_, 0, sizeof level_);
  for (int &p : pwm_) p = -1;
  memset(servo_, 0, sizeof servo_);
  memset(input_, 0xFF, sizeof input_);
  for (auto &f : isr_) f = nullptr;
  rx_.clear();
  tx_.clear();
  echo_us_ = 0;
  interrupts_on_ = true;
}
void host_setup() { setup(); }
void host_loop() { loop(); }
void host_set_micros(uint64_t us) {
  if (us > now_us) now_us = us;  // a delay() may already have taken the board past us
}
uint64_t host_micros() { return now_us; }
void host_rx(const uint8_t *data, int n) {
  for (int i = 0; i < n; ++i) rx_.push_back(data[i]);
}
int host_rx_pending() { return (int)rx_.size(); }
int host_tx(uint8_t *out, int max) {
  int n = (int)tx_.size() < max ? (int)tx_.size() : max;
  memcpy(out, tx_.data(), n);
  tx_.erase(0, n);
  return n;
}
int host_pin_mode(int pin) { return valid(pin) ? mode_[pin] : -1; }
int host_digital(int pin) { return valid(pin) ? level_[pin] : 0; }
int host_servo_us(int pin) { return valid(pin) ? servo_[pin] : 0; }
int host_pwm(int pin) {
  if (!valid(pin)) return 0;
  return pwm_[pin] >= 0 ? pwm_[pin] : (level_[pin] ? 255 : 0);
}
void host_set_input(int pin, int level) {
  if (valid(pin)) input_[pin] = level ? HIGH : LOW;
}
void host_set_echo_us(uint32_t us) { echo_us_ = us; }
void host_interrupt(int pin) {
  if (valid(pin) && isr_[pin] && interrupts_on_) isr_[pin]();
}
}
