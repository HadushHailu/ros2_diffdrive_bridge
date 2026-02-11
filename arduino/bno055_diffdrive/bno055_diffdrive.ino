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

char rx[64];
size_t rx_len = 0;
unsigned long lastCmdMs = 0;
const unsigned long TIMEOUT_MS = 300;
const int MAX_PWM = 120;

void setup() {
  pinMode(L_ENA, OUTPUT); pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(L_ENB, OUTPUT); pinMode(L_IN3, OUTPUT); pinMode(L_IN4, OUTPUT);

  pinMode(R_ENA, OUTPUT); pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);
  pinMode(R_ENB, OUTPUT); pinMode(R_IN3, OUTPUT); pinMode(R_IN4, OUTPUT);

  Serial.begin(115200);
  delay(200);
  Serial.println("# Ready. Send: M,<L>,<R>  e.g. M,150,150");

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
