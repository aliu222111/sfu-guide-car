// Drives the car's two motors through an L298N for the laptop.
//
// The car has a front and a rear drive motor and no steering. This board
// turns both the same way at the speed the laptop asks for and stops on its
// own if the laptop goes quiet. The laptop decides how far to drive
// (guide_api/straight_car.py); the board knows nothing about routes.
//
// Same serial line as guide_motor.ino, so the same tools talk to it
// (115200 baud, USB):
//   down  "C <steering -1..1> <throttle -1..1>\n"   steering is ignored;
//         throttle positive is forward. Sent at 20 Hz. No C line for 300 ms:
//         both motors stop.
//   up    "B <0|1>\n"   the button on the car changed (pressed: 1)
//         "U <cm>\n"    the front ultrasonic sensor, 10 Hz, with HAVE_ULTRASONIC
//                       (0: nothing within 4 m, -1: no sensor)
//         "# ..."       a note for a person reading the port
//
// Wiring (ESP32-WROOM dev board, L298N):
//   ENA GPIO25   IN1 GPIO26   IN2 GPIO27      motor A (front)
//   ENB GPIO13   IN3 GPIO32   IN4 GPIO33      motor B (rear)
//   button GPIO4 to ground (optional)
//   HC-SR04 (optional): trig GPIO18, echo GPIO19 through a 1k/2k divider
// Rules, in order of what breaks:
// - Pull the two jumpers off ENA and ENB on the L298N. With a jumper on, that
//   motor is always fully on and the pin cannot slow or stop it.
// - Join the grounds: L298N GND to the battery's minus AND to an ESP32 GND pin.
// - The motor battery goes to the L298N's +12V terminal, never to the ESP32.
//   The ESP32 is powered by the laptop's USB. Do not also feed it from the
//   L298N's 5V terminal.
// - A 10 k resistor from ENA to ground and one from ENB to ground keeps the
//   motors off while the board boots (its pins float until setup() runs).
// - An L298N loses about 2 V: with a 7.4 V pack the motors see about 5.4 V.

// ---- Configure for the car --------------------------------------------------
const int ENA = 25, IN1 = 26, IN2 = 27;   // motor A
const int ENB = 13, IN3 = 32, IN4 = 33;   // motor B
const bool A_REVERSED = false;            // flip if motor A turns backwards on a forward command
const bool B_REVERSED = false;            // flip if motor B turns backwards on a forward command
const int BUTTON_PIN = 4;                 // to ground, internal pull-up
const int US_TRIG = 18, US_ECHO = 19;

const int PWM_HZ = 1000;                  // an L298N switches slowly: 1 kHz keeps the torque
const int PWM_BITS = 8;                   // duty 0..255
const int MIN_DUTY = 90;                  // the smallest duty at which the car actually rolls (bench-test it)
const int MAX_DUTY = 120;                 // never more than this while moving: above it the motors and the L298N overheat
const float RAMP_PER_S = 2.5;             // throttle may change this much per second (0 to full in 0.4 s)
const unsigned long WATCHDOG_MS = 300;

// #define HAVE_ULTRASONIC
#ifdef HAVE_ULTRASONIC
const int STOP_CM = 35;                   // nearer than this ahead: no forward drive, whatever the laptop says
const unsigned long US_TIMEOUT_US = 40000;
const unsigned long US_MAX_ECHO_US = 23200;  // 4 m
unsigned long lastUs = 0;
int lastCm = 0;                           // 0: clear
#endif
// -----------------------------------------------------------------------------

static_assert(MIN_DUTY >= 0 && MIN_DUTY <= MAX_DUTY && MAX_DUTY <= 255, "duty limits: 0 <= MIN_DUTY <= MAX_DUTY <= 255");

char line[48];
uint8_t len = 0;
bool discarding = false;       // inside an overlong line: ignore until its newline
unsigned long lastCmd = 0;
unsigned long lastStep = 0;
float target = 0;              // what the laptop asked for, -1..1
float current = 0;             // what the motors are getting, after the ramp
bool lastButton = false;

float clamp1(float v) { return v > 1 ? 1 : v < -1 ? -1 : v; }

// The ESP32 Arduino core changed its PWM calls in version 3.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void pwmBegin(int pin, int) { ledcAttach(pin, PWM_HZ, PWM_BITS); }
void pwmWrite(int pin, int, int duty) { ledcWrite(pin, duty); }
#else
void pwmBegin(int pin, int channel) { ledcSetup(channel, PWM_HZ, PWM_BITS); ledcAttachPin(pin, channel); }
void pwmWrite(int, int channel, int duty) { ledcWrite(channel, duty); }
#endif

void motor(int in1, int in2, int en, int channel, float v, bool reversed) {
  if (reversed) v = -v;
  if (v == 0) {                 // coast: both inputs low, enable off
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
    pwmWrite(en, channel, 0);
    return;
  }
  digitalWrite(in1, v > 0 ? HIGH : LOW);
  digitalWrite(in2, v > 0 ? LOW : HIGH);
  float mag = v > 0 ? v : -v;
  int duty = MIN_DUTY + (int)((MAX_DUTY - MIN_DUTY) * mag + 0.5);
  pwmWrite(en, channel, duty > MAX_DUTY ? MAX_DUTY : duty);   // full throttle is MAX_DUTY, never the full 255
}

void apply(float v) {
  motor(IN1, IN2, ENA, 0, v, A_REVERSED);
  motor(IN3, IN4, ENB, 1, v, B_REVERSED);
}

void handleLine() {
  line[len] = 0;
  if (line[0] == 'C' && line[1] == ' ') {
    char *end;
    strtod(line + 2, &end);              // steering: read and ignored
    if (end == line + 2) return;
    char *end2;
    float t = strtod(end, &end2);
    if (end2 == end) return;
    if (t != t) return;                  // NaN
    target = clamp1(t);
    lastCmd = millis();
  }
}

void setup() {
  // Motors off first, before anything can hold the pins.
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  pwmBegin(ENA, 0);
  pwmBegin(ENB, 1);
  apply(0);
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
#ifdef HAVE_ULTRASONIC
  pinMode(US_TRIG, OUTPUT); digitalWrite(US_TRIG, LOW);
  pinMode(US_ECHO, INPUT);
#endif
  lastStep = millis();
  Serial.println("# guide_l298n ready");
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

  unsigned long now = millis();
  float want = (now - lastCmd > WATCHDOG_MS) ? 0 : target;
#ifdef HAVE_ULTRASONIC
  if (want > 0 && lastCm > 0 && lastCm < STOP_CM) want = 0;   // something close ahead
#endif
  // Ramp towards what is wanted; a stop is immediate.
  float dt = (now - lastStep) / 1000.0;
  lastStep = now;
  if (want == 0) {
    current = 0;
  } else {
    float step = RAMP_PER_S * dt;
    if ((current > 0 && want < 0) || (current < 0 && want > 0)) current = 0;  // never slam into reverse
    else if (want > current) current = (want - current > step) ? current + step : want;
    else current = (current - want > step) ? current - step : want;
  }
  apply(current);

  bool b = digitalRead(BUTTON_PIN) == LOW;
  if (b != lastButton) {
    delay(5);  // debounce
    if ((digitalRead(BUTTON_PIN) == LOW) == b) {
      lastButton = b;
      Serial.print("B "); Serial.println(b ? 1 : 0);
    }
  }
#ifdef HAVE_ULTRASONIC
  if (now - lastUs >= 100) {
    lastUs = now;
    digitalWrite(US_TRIG, HIGH); delayMicroseconds(10); digitalWrite(US_TRIG, LOW);
    unsigned long echo = pulseIn(US_ECHO, HIGH, US_TIMEOUT_US);  // 0: no pulse at all
    Serial.print("U ");
    if (echo == 0) { lastCm = 0; Serial.println(-1); }                    // sensor dead or unplugged
    else if (echo >= US_MAX_ECHO_US) { lastCm = 0; Serial.println(0); }   // nothing within 4 m
    else { lastCm = echo / 58; Serial.println(lastCm); }
  }
#endif
  delay(2);
}
