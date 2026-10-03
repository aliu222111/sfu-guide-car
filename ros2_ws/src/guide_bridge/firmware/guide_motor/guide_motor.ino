// Motor controller for the SFU guide car (spec section 10).
//
// Reads "V <linear m/s> <angular rad/s>\n" over USB serial at 115200 baud and
// drives a differential chassis through a two-channel H-bridge with one PWM and
// one direction pin per side (TB6612FNG, Cytron MDD10A and similar). Stops the
// motors if no V line arrives for 500 ms. Sends "B <0|1>\n" when the
// push-to-talk button changes. If encoders are fitted, define HAVE_ENCODERS and
// it sends "E <left> <right>\n" at 20 Hz.
//
// Ackermann chassis: replace mix() so angular velocity becomes a steering angle
// (spec section 10); the serial protocol stays the same.

// ---- Configure for the chassis --------------------------------------------
const float WHEEL_BASE_M = 0.30;       // distance between left and right wheels
const float MAX_WHEEL_SPEED = 1.2;     // m/s at full PWM, measure on the bench
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
// ---------------------------------------------------------------------------

char line[48];
uint8_t len = 0;
unsigned long lastCmd = 0;
bool lastButton = false;
unsigned long lastEnc = 0;

void drive(int pwmPin, int dirPin, bool invert, float speed) {
  float f = speed / MAX_WHEEL_SPEED;
  if (f > 1) f = 1;
  if (f < -1) f = -1;
  bool forward = (f >= 0) != invert;
  digitalWrite(dirPin, forward ? HIGH : LOW);
  analogWrite(pwmPin, (int)(fabs(f) * 255));
}

void mix(float v, float w) {
  drive(PWM_L, DIR_L, INVERT_L, v - w * WHEEL_BASE_M / 2);
  drive(PWM_R, DIR_R, INVERT_R, v + w * WHEEL_BASE_M / 2);
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
  stopMotors();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { handleLine(); len = 0; }
    else if (len < sizeof(line) - 1) line[len++] = c;
    else len = 0;  // overlong line: drop it
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
}
