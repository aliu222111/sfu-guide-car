// Motor controller for the SFU guide car (spec section 10).
//
// Reads "V <linear m/s> <angular rad/s>\n" over USB serial at 115200 baud and
// drives a tank-steered (skid-steer: tracks, or four wheels with each side
// driven together) or two-wheel differential chassis through a two-channel driver with one PWM and
// one direction pin per side (Cytron MDD10A, Pololu DRV8838/G2 and similar; a
// TB6612FNG or L298N needs IN1/IN2 per side, so adapt drive()). Stops the
// motors if no V line arrives for 500 ms. Sends "B <0|1>\n" when the
// push-to-talk button changes. If encoders are fitted, define HAVE_ENCODERS and
// it sends "E <left> <right>\n" at 20 Hz. With an HC-SR04 ultrasonic sensor on
// the front bumper (required with one camera, spec section 18), define
// HAVE_ULTRASONIC and it sends "U <cm>\n" at 10 Hz, 0 when nothing echoes.
//
// Tank steering: the tracks slip sideways in turns, so the car turns slower
// than this geometric mix says. The laptop's serial bridge scales the turn rate
// up (turn_gain, measured with "ros2 run guide_bridge calibrate_turn"), so
// nothing here changes when the floor does. Set TRACK_WIDTH_M to the distance
// between the track centres and MIN_PWM to the PWM at which a track just
// starts to move when the car pivots on the floor (tracks need more than
// wheels to break loose).
//
// Ackermann chassis: replace mix() so angular velocity becomes a steering angle
// (spec section 10); the serial protocol stays the same.

// ---- Configure for the chassis --------------------------------------------
const float TRACK_WIDTH_M = 0.30;      // between the centres of the left and right tracks (or wheels)
const float MAX_WHEEL_SPEED = 1.2;     // m/s at full PWM, measure on the bench
const int MIN_PWM = 60;                // 0..255: below this a side does not move; measure on the floor
const float STOP_SPEED = 0.01;         // m/s: side speeds under this are a stop, not MIN_PWM
const unsigned long WATCHDOG_MS = 500;
const int PWM_L = 5, DIR_L = 4, PWM_R = 6, DIR_R = 7;
const int BUTTON_PIN = 2;              // to ground, internal pull-up
const bool INVERT_L = false, INVERT_R = true;
// #define HAVE_ENCODERS
#ifdef HAVE_ENCODERS
const int ENC_LA = 18, ENC_LB = 19, ENC_RA = 20, ENC_RB = 21;  // interrupt pins on a Mega
volatile long ticksL = 0, ticksR = 0;
void isrL() { ticksL += digitalRead(ENC_LB) ? -1 : 1; }
void isrR() { ticksR += digitalRead(ENC_RB) ? 1 : -1; }
#endif
// #define HAVE_ULTRASONIC
#ifdef HAVE_ULTRASONIC
const int US_TRIG = 8, US_ECHO = 9;
const unsigned long US_TIMEOUT_US = 25000;  // about 4 m; also bounds how long loop() waits
unsigned long lastUs = 0;
#endif
// ---------------------------------------------------------------------------

char line[48];
uint8_t len = 0;
unsigned long lastCmd = 0;
bool discarding = false;  // inside an overlong line: ignore until its newline
bool lastButton = false;
unsigned long lastEnc = 0;

void drive(int pwmPin, int dirPin, bool invert, float speed) {
  float f = speed / MAX_WHEEL_SPEED;
  if (f > 1) f = 1;
  if (f < -1) f = -1;
  bool forward = (f >= 0) != invert;
  digitalWrite(dirPin, forward ? HIGH : LOW);
  // Map the speed onto MIN_PWM..255 so small commands still move the track.
  int pwm = fabs(speed) < STOP_SPEED ? 0 : MIN_PWM + (int)(fabs(f) * (255 - MIN_PWM));
  analogWrite(pwmPin, pwm);
}

void mix(float v, float w) {
  drive(PWM_L, DIR_L, INVERT_L, v - w * TRACK_WIDTH_M / 2);
  drive(PWM_R, DIR_R, INVERT_R, v + w * TRACK_WIDTH_M / 2);
}

void stopMotors() { mix(0, 0); }

void handleLine() {
  line[len] = 0;
  if (line[0] == 'V' && line[1] == ' ') {
    char *end;
    float v = strtod(line + 2, &end);
    if (end == line + 2) return;
    char *end2;
    float w = strtod(end, &end2);
    if (end2 == end) return;
    mix(v, w);
    lastCmd = millis();
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PWM_L, OUTPUT); pinMode(DIR_L, OUTPUT);
  pinMode(PWM_R, OUTPUT); pinMode(DIR_R, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
#ifdef HAVE_ENCODERS
  pinMode(ENC_LA, INPUT_PULLUP); pinMode(ENC_LB, INPUT_PULLUP);
  pinMode(ENC_RA, INPUT_PULLUP); pinMode(ENC_RB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_LA), isrL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RA), isrR, RISING);
#endif
#ifdef HAVE_ULTRASONIC
  pinMode(US_TRIG, OUTPUT); digitalWrite(US_TRIG, LOW);
  pinMode(US_ECHO, INPUT);
#endif
  stopMotors();
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
  if (millis() - lastCmd > WATCHDOG_MS) stopMotors();

  bool b = digitalRead(BUTTON_PIN) == LOW;
  if (b != lastButton) {
    delay(5);  // debounce
    if ((digitalRead(BUTTON_PIN) == LOW) == b) {
      lastButton = b;
      Serial.print("B "); Serial.println(b ? 1 : 0);
    }
  }
#ifdef HAVE_ENCODERS
  if (millis() - lastEnc >= 50) {
    lastEnc = millis();
    noInterrupts(); long l = ticksL, r = ticksR; interrupts();
    Serial.print("E "); Serial.print(l); Serial.print(' '); Serial.println(r);
  }
#endif
#ifdef HAVE_ULTRASONIC
  if (millis() - lastUs >= 100) {
    lastUs = millis();
    digitalWrite(US_TRIG, HIGH); delayMicroseconds(10); digitalWrite(US_TRIG, LOW);
    unsigned long echo = pulseIn(US_ECHO, HIGH, US_TIMEOUT_US);  // 0 on timeout
    Serial.print("U "); Serial.println(echo / 58);                // microseconds to cm
  }
#endif
}
