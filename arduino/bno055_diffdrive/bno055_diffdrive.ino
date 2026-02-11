#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// ===== Left L298N =====
const int L_ENA = 5;   // PWM
const int L_IN1 = 8;
const int L_IN2 = 7;
const int L_ENB = 6;   // PWM
const int L_IN3 = 10;
const int L_IN4 = 11;

// ===== Right L298N =====
const int R_ENA = 3;   // PWM
const int R_IN1 = 2;
const int R_IN2 = 4;
const int R_ENB = 9;   // PWM
const int R_IN3 = 12;
const int R_IN4 = 13;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
bool imu_ok = false;

char rx[64];
size_t rx_len = 0;
unsigned long lastCmdMs = 0;
const unsigned long TIMEOUT_MS = 300;
const int MAX_PWM = 120;
const unsigned long IMU_PERIOD_MS = 20; // 50 Hz
unsigned long lastImuMs = 0;

void setup() {
  pinMode(L_ENA, OUTPUT); pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(L_ENB, OUTPUT); pinMode(L_IN3, OUTPUT); pinMode(L_IN4, OUTPUT);

  pinMode(R_ENA, OUTPUT); pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);
  pinMode(R_ENB, OUTPUT); pinMode(R_IN3, OUTPUT); pinMode(R_IN4, OUTPUT);

  Serial.begin(115200);
  delay(200);
  Serial.println("# Ready. Send: M,<L>,<R>  e.g. M,150,150");

  // Use ACCGYRO mode for raw accel+gyro (no internal fusion)
  // This is what VIO systems need - not NDOF which does internal fusion
  imu_ok = bno.begin(Adafruit_BNO055::OPERATION_MODE_ACCGYRO);
  if (imu_ok) {
    bno.setExtCrystalUse(true);
    Serial.println("# BNO055 OK (ACCGYRO mode)");
  } else {
    Serial.println("# BNO055 not detected");
  }

  stopMotors();
  lastCmdMs = millis();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (rx_len > 0) {
        rx[rx_len] = '\0';
        handle(rx);
        rx_len = 0;
      }
    } else {
      if (rx_len < sizeof(rx) - 1) {
        rx[rx_len++] = c;
      } else {
        rx_len = 0; // overflow guard
      }
    }
  }

  if (millis() - lastCmdMs > TIMEOUT_MS) {
    stopMotors();
  }

  if (imu_ok && (millis() - lastImuMs >= IMU_PERIOD_MS)) {
    publishImu();
    lastImuMs = millis();
  }
}

void handle(const char* line) {
  int L = 0;
  int R = 0;
  if (sscanf(line, "M,%d,%d", &L, &R) != 2) return;

  L = constrain(L, -MAX_PWM, MAX_PWM);
  R = constrain(R, -MAX_PWM, MAX_PWM);

  setSide(true, L);
  setSide(false, R);

  lastCmdMs = millis();

  // Optional: add debug prints if needed
}

void publishImu() {
  unsigned long t_us = micros();  // capture timestamp first
  // Use VECTOR_ACCELEROMETER (raw with gravity), NOT VECTOR_LINEARACCEL (gravity removed)
  // VIO systems need gravity for orientation reference
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Quaternion quat = bno.getQuat();

  // Format: I,<timestamp_us>,ax,ay,az,gx,gy,gz,qw,qx,qy,qz
  Serial.print("I,");
  Serial.print(t_us);            Serial.print(',');
  Serial.print(accel.x(), 6);    Serial.print(',');
  Serial.print(accel.y(), 6);    Serial.print(',');
  Serial.print(accel.z(), 6);    Serial.print(',');
  Serial.print(gyro.x(), 6);     Serial.print(',');
  Serial.print(gyro.y(), 6);     Serial.print(',');
  Serial.print(gyro.z(), 6);     Serial.print(',');
  Serial.print(quat.w(), 6);     Serial.print(',');
  Serial.print(quat.x(), 6);     Serial.print(',');
  Serial.print(quat.y(), 6);     Serial.print(',');
  Serial.print(quat.z(), 6);
  Serial.println();
}

void stopMotors() {
  analogWrite(L_ENA, 0); analogWrite(L_ENB, 0);
  analogWrite(R_ENA, 0); analogWrite(R_ENB, 0);

  // coast
  digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, LOW);
  digitalWrite(L_IN3, LOW); digitalWrite(L_IN4, LOW);
  digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, LOW);
  digitalWrite(R_IN3, LOW); digitalWrite(R_IN4, LOW);
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
    analogWrite(L_ENA, p); analogWrite(L_ENB, p);
  } else {
    if (pwm >= 0) {
      digitalWrite(R_IN1, HIGH); digitalWrite(R_IN2, LOW);
      digitalWrite(R_IN3, HIGH); digitalWrite(R_IN4, LOW);
    } else {
      digitalWrite(R_IN1, LOW);  digitalWrite(R_IN2, HIGH);
      digitalWrite(R_IN3, LOW);  digitalWrite(R_IN4, HIGH);
    }
    analogWrite(R_ENA, p); analogWrite(R_ENB, p);
  }
}
