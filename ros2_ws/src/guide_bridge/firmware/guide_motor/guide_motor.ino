// Works the RC car's hand controller for the laptop (spec section 10).
//
// The car is a stock RC car (Ackermann steering, four-wheel drive) and its
// stock pistol-grip controller. This board stands in for the controller's
// steering wheel and trigger: their potentiometer wipers are disconnected and
// these two outputs drive the wiper wires instead, so the controller radios the
// car as if someone held it. Reads "C <steering -1..1> <throttle -1..1>\n" over
// USB serial at 115200 baud (positive: left, forward); the laptop does all the
// car's maths. Centres both (straight, no throttle) if no C line arrives for
// 300 ms. Sends "B <0|1>\n" when the push-to-talk button changes. With an
// HC-SR04 ultrasonic sensor on the front bumper (required with one camera,
// spec section 18), define HAVE_ULTRASONIC and it sends "U <cm>\n" at 10 Hz:
// 0 when nothing is within 4 m, -1 when the sensor gives no echo at all.
// (Sensors on the car need the board on the car; the controller can ride on
// the car too, its antenna clear of the laptop.)
//
// The board is an ESP32-WROOM-32E: its two DAC pins, GPIO25 (steering) and
// GPIO26 (throttle), drive the wiper wires directly. Compiled for another
// board (or the simulator) it uses PWM pins, each through an RC filter
// (10 kOhm, 10 uF) to the wiper wire. Arduino IDE: board "ESP32 Dev Module".
// Measure on the controller with a multimeter, powered, the wiper connected:
// - the wiper voltage with the wheel and trigger at rest (STEER_CENTER_V,
//   THROTTLE_CENTER_V);
// - at full left and full trigger (STEER_LEFT_V, THROTTLE_FULL_V).
// Then set the trims, deadband and speed in guide.yaml (README, "The RC car").
// Tie the board's ground to the controller's ground; never feed more than
// the controller's own supply voltage into it.

// ---- Configure for the controller -------------------------------------------
const float VREF = 3.3;                // the board's output range, 0..VREF
const float STEER_CENTER_V = 1.65;     // wheel at rest
const float STEER_LEFT_V = 0.35;       // wheel at full left
const float THROTTLE_CENTER_V = 1.65;  // trigger at rest
const float THROTTLE_FULL_V = 2.95;    // trigger pulled fully (forward)
const unsigned long WATCHDOG_MS = 300;
#if defined(ARDUINO_ARCH_ESP32)
#define USE_DAC
const int STEER_PIN = 25, THROTTLE_PIN = 26;  // DAC1, DAC2
const int BUTTON_PIN = 4;                     // to ground, internal pull-up (not GPIO2: a boot pin)
const int US_TRIG = 18, US_ECHO = 19;         // the HC-SR04's 5 V echo through a divider (1k/2k) to 3.3 V
#else
const int STEER_PIN = 9, THROTTLE_PIN = 10;   // PWM, through an RC filter
const int BUTTON_PIN = 2;
const int US_TRIG = 11, US_ECHO = 12;
#endif
// #define HAVE_ULTRASONIC
#ifdef HAVE_ULTRASONIC
// An HC-SR04 with nothing in range still sends a ~38 ms echo pulse; a dead one
// sends none. Waiting past 38 ms tells the two apart (loop() blocks up to 40 ms).
const unsigned long US_TIMEOUT_US = 40000;
const unsigned long US_MAX_ECHO_US = 23200;  // 4 m: longer pulses mean nothing in range
unsigned long lastUs = 0;
#endif
// -----------------------------------------------------------------------------

char line[48];
uint8_t len = 0;
unsigned long lastCmd = 0;
bool discarding = false;  // inside an overlong line: ignore until its newline
bool lastButton = false;

float clamp1(float v) { return v > 1 ? 1 : v < -1 ? -1 : v; }

void writeVolts(int pin, float volts) {
  if (volts < 0) volts = 0;
  if (volts > VREF) volts = VREF;
  int level = (int)(volts / VREF * 255 + 0.5);
#ifdef USE_DAC
  dacWrite(pin, level);
#else
  analogWrite(pin, level);
#endif
}

void drive(float steering, float throttle) {
  // Linear between the measured end points; the far side mirrors them.
  writeVolts(STEER_PIN, STEER_CENTER_V + clamp1(steering) * (STEER_LEFT_V - STEER_CENTER_V));
  writeVolts(THROTTLE_PIN, THROTTLE_CENTER_V + clamp1(throttle) * (THROTTLE_FULL_V - THROTTLE_CENTER_V));
}

void handleLine() {
  line[len] = 0;
  if (line[0] == 'C' && line[1] == ' ') {
    char *end;
    float s = strtod(line + 2, &end);
    if (end == line + 2) return;
    char *end2;
    float t = strtod(end, &end2);
    if (end2 == end) return;
    drive(s, t);
    lastCmd = millis();
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(STEER_PIN, OUTPUT);
  pinMode(THROTTLE_PIN, OUTPUT);
  drive(0, 0);  // centred before the controller is switched on
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
  if (millis() - lastCmd > WATCHDOG_MS) drive(0, 0);

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
