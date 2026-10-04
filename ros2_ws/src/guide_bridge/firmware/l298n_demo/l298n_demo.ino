// Motors straight from an L298N (or similar) driver, for a demo while the steering is broken.
//
// Same serial protocol as guide_motor.ino, so the laptop's tools drive it unchanged
// (voice_check --drive, camera_drive_demo, esp32_drive_test): "C <steering -1..1>
// <throttle -1..1>\n" at 115200 baud. Throttle sets both motors' speed and direction;
// steering, if STEER_MIX is above 0, slows one side to turn (two motors, left and right).
// Stops both motors if no C line arrives for 300 ms (laptop crash, cable pulled).
//
// Wiring: copy the pin numbers from your motor test sketch below. Keep motor pins off
// GPIO0, 2, 12 and 15: the ESP32 reads those at power-on, and a driver input on one stops
// uploads ("Wrong boot mode detected"). Tie the driver's GND to the ESP32's GND; power the
// motors from the battery, not the ESP32. Arduino IDE: board "ESP32 Dev Module".

// ---- Configure ---------------------------------------------------------------
// The car's wiring (ENA/ENB jumpers off, L298N GND to ESP32 GND):
const int ENA = 25, IN1 = 26, IN2 = 27;  // motor A (OUT1/OUT2): speed (PWM) and direction
const int ENB = 13, IN3 = 33, IN4 = 32;  // motor B (OUT3/OUT4)
const int MIN_PWM = 90;          // below this the motors hum but do not turn: start here
const float STEER_MIX = 0.0;     // 0: steering ignored (straight only); 0.5: turn by slowing one side
// Forward on this car is IN1 high (26) and IN4 high (32), as the motor test drove it:
// motor B is wired the other way round, so it is flipped. A motor running backwards: toggle.
const bool FLIP_A = false, FLIP_B = true;
const unsigned long WATCHDOG_MS = 300;
// -----------------------------------------------------------------------------

char line[48];
uint8_t len = 0;
unsigned long lastCmd = 0;
bool discarding = false;

float clamp1(float v) { return v > 1 ? 1 : v < -1 ? -1 : v; }

void motor(int en, int a, int b, float v, bool flip) {
  v = clamp1(flip ? -v : v);
  int pwm = (v == 0) ? 0 : MIN_PWM + (int)((255 - MIN_PWM) * (v < 0 ? -v : v));
  digitalWrite(a, v > 0 ? HIGH : LOW);
  digitalWrite(b, v < 0 ? HIGH : LOW);
  analogWrite(en, pwm);
}

void drive(float steering, float throttle) {
  float s = clamp1(steering) * STEER_MIX, t = clamp1(throttle);
  motor(ENA, IN1, IN2, t * (1 - (s > 0 ? s : 0)), FLIP_A);   // left side slows to turn left (+)
  motor(ENB, IN3, IN4, t * (1 - (s < 0 ? -s : 0)), FLIP_B);  // right side slows to turn right
}

void handle(char *l) {
  if (l[0] != 'C' || l[1] != ' ') return;
  char *end;
  float s = strtod(l + 2, &end);
  if (end == l + 2) return;
  char *end2;
  float t = strtod(end, &end2);
  if (end2 == end) return;
  drive(s, t);
  lastCmd = millis();
}

void setup() {
  Serial.begin(115200);
  int pins[] = {ENA, IN1, IN2, ENB, IN3, IN4};
  for (int p : pins) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  drive(0, 0);
  Serial.println("l298n_demo ready");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (!discarding && len) { line[len] = 0; handle(line); }
      len = 0; discarding = false;
    } else if (len < sizeof(line) - 1) {
      line[len++] = c;
    } else {
      discarding = true;
    }
  }
  if (millis() - lastCmd > WATCHDOG_MS) drive(0, 0);
}
