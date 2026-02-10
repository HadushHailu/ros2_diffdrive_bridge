#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// ===== Left L298N =====
const int L_ENA = 5;   // PWM
const int L_IN1 = 8;
const int L_IN2 = 9;
const int L_ENB = 6;   // PWM
const int L_IN3 = 10;
const int L_IN4 = 11;

// ===== Right L298N =====
const int R_ENA = 3;   // PWM
const int R_IN1 = 2;
const int R_IN2 = 4;
const int R_ENB = 7;   // PWM
const int R_IN3 = 12;
const int R_IN4 = 13;

// ---- motor safety ----
unsigned long lastCmdMs = 0;
const unsigned long CMD_TIMEOUT_MS = 300;

String rxLine = "";
int curL = 0;
int curR = 0;

const int MAX_PWM = 255;
const int MAX_STEP = 12;   // smooth ramping

// ---- IMU timing ----
unsigned long lastImuPrint = 0;

void setup() {
  // Motor pins
  pinMode(L_ENA, OUTPUT); pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(L_ENB, OUTPUT); pinMode(L_IN3, OUTPUT); pinMode(L_IN4, OUTPUT);
  pinMode(R_ENA, OUTPUT); pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);
  pinMode(R_ENB, OUTPUT); pinMode(R_IN3, OUTPUT); pinMode(R_IN4, OUTPUT);

  stopMotors();

  Serial.begin(115200);
  delay(300);

  Serial.println("# Starting BNO055...");
  if (!bno.begin()) {
    Serial.println("# ERROR: BNO055 not detected");
    while (1) {}
  }

  bno.setExtCrystalUse(true);
  Serial.println("# OK: BNO055 ready");
  Serial.println("# IMU: t_ms, ax, ay, az, gx, gy, gz, qw, qx, qy, qz");
  Serial.println("# Motor cmd: M,<L>,<R>");

  lastCmdMs = millis();
}

void loop() {
  // ---- SERIAL COMMAND PARSER ----
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      rxLine.trim();
      if (rxLine.length() > 0) handleCommand(rxLine);
      rxLine = "";
    } else {
      rxLine += c;
    }
  }

  // ---- DEADMAN STOP ----
  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    setMotors(0, 0);
  }

  // ---- IMU STREAM @ 50 Hz ----
  unsigned long now = millis();
  if (now - lastImuPrint >= 20) {
    lastImuPrint = now;

    imu::Vector<3> la = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> g  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Quaternion q = bno.getQuat();

    Serial.print(now); Serial.print(",");
    Serial.print(la.x()); Serial.print(",");
    Serial.print(la.y()); Serial.print(",");
    Serial.print(la.z()); Serial.print(",");
    Serial.print(g.x());  Serial.print(",");
    Serial.print(g.y());  Serial.print(",");
    Serial.print(g.z());  Serial.print(",");
    Serial.print(q.w(), 6); Serial.print(",");
    Serial.print(q.x(), 6); Serial.print(",");
    Serial.print(q.y(), 6); Serial.print(",");
    Serial.println(q.z(), 6);
  }
}

// ================= MOTOR CONTROL =================

void handleCommand(const String& line) {
  // Expected: M,<L>,<R>
  if (!line.startsWith("M,")) return;

  int c1 = line.indexOf(',', 2);
  if (c1 < 0) return;

  int L = line.substring(2, c1).toInt();
  int R = line.substring(c1 + 1).toInt();

  L = constrain(L, -MAX_PWM, MAX_PWM);
  R = constrain(R, -MAX_PWM, MAX_PWM);

  curL = ramp(curL, L);
  curR = ramp(curR, R);

  setMotors(curL, curR);
  lastCmdMs = millis();
}

int ramp(int cur, int target) {
  if (target > cur + MAX_STEP) return cur + MAX_STEP;
  if (target < cur - MAX_STEP) return cur - MAX_STEP;
  return target;
}

void stopMotors() {
  setMotors(0, 0);
}

void setMotors(int pwmL, int pwmR) {
  setSide(true, pwmL);
  setSide(false, pwmR);
}

void setSide(bool left, int pwm) {
  int p = abs(pwm);

  if (left) {
    if (pwm >= 0) {
      digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, LOW);
      digitalWrite(L_IN3, HIGH); digitalWrite(L_IN4, LOW);
    } else {
      digitalWrite(L_IN1, LOW);  digitalWrite(L_IN2, HIGH);
      digitalWrite(L_IN3, LOW);  digitalWrite(L_IN4, HIGH);
    }
    analogWrite(L_ENA, p);
    analogWrite(L_ENB, p);
  } else {
    if (pwm >= 0) {
      digitalWrite(R_IN1, HIGH); digitalWrite(R_IN2, LOW);
      digitalWrite(R_IN3, HIGH); digitalWrite(R_IN4, LOW);
    } else {
      digitalWrite(R_IN1, LOW);  digitalWrite(R_IN2, HIGH);
      digitalWrite(R_IN3, LOW);  digitalWrite(R_IN4, HIGH);
    }
    analogWrite(R_ENA, p);
    analogWrite(R_ENB, p);
  }
}
