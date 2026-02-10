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

String rx = "";
unsigned long lastCmdMs = 0;
const unsigned long TIMEOUT_MS = 300;

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
    if (c == '\n') {
      rx.trim();
      if (rx.length() > 0) handle(rx);
      rx = "";
    } else {
      rx += c;
    }
  }

  if (millis() - lastCmdMs > TIMEOUT_MS) {
    stopMotors();
  }
}

void handle(const String& line) {
  Serial.println("# GOT CMD");
  if (!line.startsWith("M,")) return;

  int c1 = line.indexOf(',', 2);
  if (c1 < 0) return;

  int L = line.substring(2, c1).toInt();
  int R = line.substring(c1 + 1).toInt();

  L = constrain(L, -255, 255);
  R = constrain(R, -255, 255);

  setSide(true, L);
  setSide(false, R);

  lastCmdMs = millis();

  // Debug: prove Arduino parsed it
  Serial.print("# CMD "); Serial.print(L); Serial.print(" "); Serial.println(R);
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
