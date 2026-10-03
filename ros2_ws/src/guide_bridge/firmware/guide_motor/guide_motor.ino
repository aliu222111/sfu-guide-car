// Motor controller for the SFU guide car (spec section 10).
//
// The car is an RC car with Ackermann steering and four-wheel drive: a steering
// servo and an ESC, each driven by a standard 50 Hz servo pulse (Servo library).
// Reads "V <linear m/s> <angular rad/s>\n" over USB serial at 115200 baud and
// turns it into a throttle pulse and a steering angle with the bicycle model,
// steer = atan(WHEELBASE * angular / linear). A car cannot turn without moving:
// asked to turn while stopped, it points the wheels and waits. Goes to neutral
// if no V line arrives for 500 ms. Sends "B <0|1>\n" when the push-to-talk
// button changes. With an HC-SR04 ultrasonic sensor on the front bumper
// (required with one camera, spec section 18), define HAVE_ULTRASONIC and it
// sends "U <cm>\n" at 10 Hz: 0 when nothing is within 4 m, -1 when the sensor
// gives no echo pulse at all (dead or unplugged; the laptop then treats the
// bumper as silent and the car creeps).
//
// Measure on the floor (spec section 10, README runbook):
// - STEER_CENTER_US: the pulse at which the car rolls straight (trim).
// - MAX_STEER_RAD and STEER_LOCK_US: drive a slow full-lock circle; its radius
//   R gives MAX_STEER_RAD = atan(WHEELBASE_M / R).
// - ESC_START_US: the smallest step from neutral at which the car rolls.
// - SPEED_AT_LIMIT: its speed at ESC_LIMIT_US (time it over 5 m).
// RC cars are fast: ESC_LIMIT_US caps the throttle, whatever the laptop asks.
// Power the servo from the ESC's BEC or a 6 V supply, never the Arduino's 5 V
// pin; tie the grounds together.

#include <Servo.h>

// ---- Configure for the chassis --------------------------------------------
const float WHEELBASE_M = 0.48;        // front to rear axle: measure yours
const float MAX_STEER_RAD = 0.52;      // front wheel angle at full lock (about 30 deg): measure
const int STEER_CENTER_US = 1500;      // trim: the car rolls straight
const int STEER_LOCK_US = 400;         // pulse change from centre to full lock
const bool STEER_REVERSED = false;     // true if a longer pulse steers right
const int ESC_NEUTRAL_US = 1500;
const int ESC_START_US = 60;           // beyond neutral: the car just starts to roll
const int ESC_LIMIT_US = 180;          // never further from neutral than this
const float SPEED_AT_LIMIT = 1.6;      // m/s at ESC_LIMIT_US on the floor
const bool ESC_REVERSED = false;       // true if a longer pulse drives backwards
const float STOP_SPEED = 0.02;         // m/s: slower requests are a stop
const unsigned long WATCHDOG_MS = 500;
const int STEER_PIN = 9, ESC_PIN = 10; // any pins; the Servo library uses Timer1
const int BUTTON_PIN = 2;              // to ground, internal pull-up
// Many car ESCs brake on the first reverse pulse and reverse only after a
// return to neutral. Define this for those (not for crawler ESCs in
// forward/reverse mode): reversing then starts with brake, neutral, reverse.
// #define ESC_BRAKE_BEFORE_REVERSE
// #define HAVE_ULTRASONIC
#ifdef HAVE_ULTRASONIC
const int US_TRIG = 11, US_ECHO = 12;
// An HC-SR04 with nothing in range still sends a ~38 ms echo pulse; a dead one
// sends none. Waiting past 38 ms tells the two apart (loop() blocks up to 40 ms).
const unsigned long US_TIMEOUT_US = 40000;
const unsigned long US_MAX_ECHO_US = 23200;  // 4 m: longer pulses mean nothing in range
unsigned long lastUs = 0;
#endif
// ---------------------------------------------------------------------------

Servo steer, esc;
char line[48];
uint8_t len = 0;
unsigned long lastCmd = 0;
bool discarding = false;  // inside an overlong line: ignore until its newline
bool lastButton = false;
float targetV = 0, targetW = 0;
float steerRad = 0;       // held while stopped, so the car starts into a turn
#ifdef ESC_BRAKE_BEFORE_REVERSE
bool reverseReady = false;
unsigned long reverseAskedAt = 0;
#endif

int escPulse(float v) {
  float f = fabs(v) / SPEED_AT_LIMIT;
  if (f > 1) f = 1;
  int step = ESC_START_US + (int)(f * (ESC_LIMIT_US - ESC_START_US));
  bool forward = (v > 0) != ESC_REVERSED;
  return ESC_NEUTRAL_US + (forward ? step : -step);
}

void setThrottle(float v) {
  if (fabs(v) < STOP_SPEED) {
    esc.writeMicroseconds(ESC_NEUTRAL_US);
    return;
  }
#ifdef ESC_BRAKE_BEFORE_REVERSE
  if (v > 0) {
    reverseReady = false;
  } else if (!reverseReady) {
    if (reverseAskedAt == 0) reverseAskedAt = millis() | 1;
    unsigned long t = millis() - reverseAskedAt;
    if (t < 150) { esc.writeMicroseconds(escPulse(-STOP_SPEED)); return; }  // brake
    if (t < 300) { esc.writeMicroseconds(ESC_NEUTRAL_US); return; }        // back to neutral
    reverseReady = true;
    reverseAskedAt = 0;
  }
#endif
  esc.writeMicroseconds(escPulse(v));
}

void setSteering(float v, float w) {
  if (fabs(v) >= STOP_SPEED) {
    steerRad = atan(WHEELBASE_M * w / v);  // reversing: the same formula steers the right way
  } else if (fabs(w) > 0.05) {
    steerRad = w > 0 ? MAX_STEER_RAD : -MAX_STEER_RAD;  // asked to turn on the spot: point the wheels
  }
  if (steerRad > MAX_STEER_RAD) steerRad = MAX_STEER_RAD;
  if (steerRad < -MAX_STEER_RAD) steerRad = -MAX_STEER_RAD;
  int us = (int)(steerRad / MAX_STEER_RAD * STEER_LOCK_US);  // positive steers left
  steer.writeMicroseconds(STEER_CENTER_US + (STEER_REVERSED ? us : -us));
}

void handleLine() {
  line[len] = 0;
  if (line[0] == 'V' && line[1] == ' ') {
    char *end;
    float v = strtod(line + 2, &end);
    if (end == line + 2) return;
    char *end2;
    float w = strtod(end, &end2);
    if (end2 == end) return;
    targetV = v;
    targetW = w;
    lastCmd = millis();
  }
}

void setup() {
  Serial.begin(115200);
  steer.attach(STEER_PIN);
  esc.attach(ESC_PIN);
  steer.writeMicroseconds(STEER_CENTER_US);
  esc.writeMicroseconds(ESC_NEUTRAL_US);  // ESCs arm on a steady neutral pulse
  pinMode(BUTTON_PIN, INPUT_PULLUP);
#ifdef HAVE_ULTRASONIC
  pinMode(US_TRIG, OUTPUT); digitalWrite(US_TRIG, LOW);
  pinMode(US_ECHO, INPUT);
#endif
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (!discarding) handleLine();
      len = 0;
      discarding = false;
    } else if (discarding) {
      // drop the rest of an overlong line so it cannot become a command
    } else if (len < sizeof(line) - 1) {
      line[len++] = c;
    } else {
      len = 0;
      discarding = true;
    }
  }
  if (millis() - lastCmd > WATCHDOG_MS) targetV = targetW = 0;
  setSteering(targetV, targetW);
  setThrottle(targetV);

  bool b = digitalRead(BUTTON_PIN) == LOW;
  if (b != lastButton) {
    delay(5);  // debounce
    if ((digitalRead(BUTTON_PIN) == LOW) == b) {
      lastButton = b;
      Serial.print("B "); Serial.println(b ? 1 : 0);
    }
  }
#ifdef HAVE_ULTRASONIC
  if (millis() - lastUs >= 100) {
    lastUs = millis();
    digitalWrite(US_TRIG, HIGH); delayMicroseconds(10); digitalWrite(US_TRIG, LOW);
    unsigned long echo = pulseIn(US_ECHO, HIGH, US_TIMEOUT_US);  // 0: no pulse at all
    Serial.print("U ");
    if (echo == 0) Serial.println(-1);                       // sensor dead or unplugged
    else if (echo >= US_MAX_ECHO_US) Serial.println(0);      // nothing within 4 m
    else Serial.println(echo / 58);                          // microseconds to cm
  }
#endif
}
