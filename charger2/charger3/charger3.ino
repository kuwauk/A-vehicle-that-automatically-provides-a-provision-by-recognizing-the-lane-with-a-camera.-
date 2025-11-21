/*
  순차 동작 제어 (L9110 + L298N + 초음파 + IR 센서)
  1. RED LED 켜짐
  2. 버튼(3) 누르면 모터2 작동
  3. IR 센서 검은색 감지(LOW) → 모터2 정지 → 모터1 작동
  4. 초음파 <= 3cm → 모터1 정지 → GREEN LED 점등
  5. 버튼(6) 누르면 역순 복귀 동작 수행 (모터1→2 역방향)
*/

#include <Arduino.h>

// --- 핀 정의 ---
const int BUTTON_PIN = 3;
const int RETURN_BUTTON_PIN = 6; // 되돌리기 버튼
const int IR1_PIN = 2;           // IR 센서 (모터2 정지)
const int TRIG = 12;
const int ECHO = 4;

const int RED_LED = 7;
const int GREEN_LED = 13;

// --- 모터1 (L298N) ---
const int DC1_EN = A0;
const int DC1_IN1 = A1;
const int DC1_IN2 = A2;

// --- 모터2 (L9110) ---
const int M2_IN1 = 8;
const int M2_IN2 = 9;

// --- 상수 ---
const int MOTOR_SPEED = 200;
const unsigned long DEBOUNCE_MS = 50;
const unsigned long REVERSE_DELAY = 800; // 1초 역방향 가동

// --- 상태 정의 ---
enum State { IDLE, MOTOR2_RUNNING, MOTOR1_RUNNING, COMPLETE };
State stateMachine = IDLE;

// --- 변수 ---
unsigned long lastButtonChange = 0;
unsigned long motor1StartTime = 0;
unsigned long motor2StartTime = 0;


void setup() {
  Serial.begin(9600);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(RETURN_BUTTON_PIN, INPUT_PULLUP);
  pinMode(IR1_PIN, INPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  pinMode(DC1_EN, OUTPUT);
  pinMode(DC1_IN1, OUTPUT);
  pinMode(DC1_IN2, OUTPUT);

  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);
  stopAllMotors();

  Serial.println("대기 중 (RED LED ON) - 버튼을 눌러 시작합니다.");
}

void loop() {

  // 메인 시작 버튼
  if (digitalRead(BUTTON_PIN) == LOW && (millis() - lastButtonChange) > DEBOUNCE_MS) {
    if (stateMachine == IDLE) {
      Serial.println("버튼 입력: 모터2 시작");
      startMotor2_L9110();
      stateMachine = MOTOR2_RUNNING;
    }
    lastButtonChange = millis();
  }

  // 되돌리기 버튼
  if (digitalRead(RETURN_BUTTON_PIN) == LOW && (millis() - lastButtonChange) > DEBOUNCE_MS) {
    Serial.println("되돌리기 버튼 입력 - 역방향 동작 시작");
    reverseSequence();
    stateMachine = IDLE;
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, HIGH);
    lastButtonChange = millis();
  }

  // 상태 제어
  switch (stateMachine) {
    case IDLE:
      break;

    // 🔹 먼저 모터2 동작
    case MOTOR2_RUNNING: {
      int irValue1 = digitalRead(IR1_PIN);

      // 1️⃣ IR 감지 → MOTOR2 정지 후 MOTOR1 시작
      if (irValue1 == LOW) {
        Serial.println("IR 감지 → MOTOR1 RUN");
        stopMotor2_L9110();
        startMotor1();
        motor2StartTime = millis();   // MOTOR1 타이머 시작
        stateMachine = MOTOR1_RUNNING;
        break;
      }

      // 2️⃣ MOTOR2 타임아웃 체크 (예: 3초)
      if (millis() - motor2StartTime > 2500) {
        Serial.println("MOTOR2 타임아웃 → 강제 정지 후 MOTOR1 시작");
        stopMotor2_L9110();
        startMotor1();
        stateMachine = MOTOR1_RUNNING;

        break;
      }

      break;
}


    // 🔹 모터1 동작
    case MOTOR1_RUNNING: {
      float distance = measureDistance();

      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");

  // ✅ 정상 조건: 4cm 이하 → 정지
      if (distance > 0 && distance <= 3.5) {
        Serial.println("4cm 이하 감지 → MOTOR1 정지 → GREEN LED ON");
        stopMotor1();
        digitalWrite(RED_LED, LOW);
        digitalWrite(GREEN_LED, HIGH);
        stateMachine = COMPLETE;
        break;
        }

  // ✅ 타임아웃 조건 (2초 초과)
      if (millis() - motor1StartTime > 2000) {
        Serial.println("타임아웃 → 강제 정지");
        stopMotor1();
        stopMotor2_L9110();
        digitalWrite(RED_LED, LOW);
        digitalWrite(GREEN_LED, HIGH);
        stateMachine = COMPLETE;
        break;
        }
      break;
    }


    case COMPLETE:
      stopAllMotors();
      break;
  }
  

  delay(50);
}

// ---------------- 되돌리기 동작 ----------------
// ---------------- 되돌리기 동작 (센서 기반) ----------------
void reverseSequence() {
  Serial.println("되돌리기 동작 시작 - 초음파 기반 제어");

  // 1️⃣ 모터1 역방향 구동 시작
  reverseMotor1();
  Serial.println("모터1 역방향 가동 중... (3.5cm 이하 시 정지)");

  unsigned long startTime = millis();
  bool motor1Stopped = false;

  while (true) {
    float distance = measureDistance();

    // 디버깅용 거리 출력
    Serial.print("거리: ");
    Serial.print(distance);
    Serial.println("cm");

    // 초음파가 3.5cm 이하로 내려가면 모터1 정지
    if (!motor1Stopped && distance-0.4 >= 5) {
      Serial.println("5cm 이상 감지 → 모터1 정지");
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");
      stopMotor1();
      motor1Stopped = true;

      // 1️⃣.2초 후 모터2 정지
      Serial.println("모터2 역방향 시작");
      reverseMotor2_L9110();
      delay(2200);
      stopMotor2_L9110();
      Serial.println("모터2 정지 완료");
      break;
    }

    // 안전 타임아웃 (예: 5초)
    if (millis() - startTime > 15000) {
      Serial.println("타임아웃 → 되돌리기 강제 정지");
      stopMotor1();
      stopMotor2_L9110();
      break;
    }

    delay(50); // 센서 안정화 간격
  }

  Serial.println("역방향 동작 완료 → 초기 상태 복귀");
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, HIGH);
}


// ---------------- 모터 제어 함수 ----------------
void startMotor1() {
  digitalWrite(DC1_IN1, HIGH);
  digitalWrite(DC1_IN2, LOW);
  analogWrite(DC1_EN, MOTOR_SPEED);
}

void stopMotor1() {
  analogWrite(DC1_EN, 0);
}

void reverseMotor1() {
  digitalWrite(DC1_IN1, LOW);
  digitalWrite(DC1_IN2, HIGH);
  analogWrite(DC1_EN, MOTOR_SPEED);
}

void startMotor2_L9110() {
  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, HIGH);
}

void stopMotor2_L9110() {
  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, LOW);
}

void reverseMotor2_L9110() {
  digitalWrite(M2_IN1, HIGH);
  digitalWrite(M2_IN2, LOW);
}

void stopAllMotors() {
  stopMotor1();
  stopMotor2_L9110();
}

// ---------------- 초음파 거리 측정 함수 ----------------
float measureDistance() {
  long duration;
  float distance;
  float sum = 0;
  int validCount = 0;

  for (int i = 0; i < 3; i++) {
    // Trigger LOW
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);

    // Trigger HIGH
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    // Echo 측정 (타임아웃 20ms → 약 3.4m 상한)
    duration = pulseIn(ECHO, HIGH, 20000);

    // 무반사 시 스킵
    if (duration > 0) {
      distance = (duration * 0.0343) / 2.0;
      sum += distance;
      validCount++;
    }

    delay(10); // 센서 간 안정화
  }

  if (validCount == 0) return -1; // 감지 실패 시 -1

  return sum / validCount; // 평균값 반환
}

