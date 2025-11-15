#include <Arduino.h>
#include <Servo.h>

Servo steering;
Servo esc;  // ✅ ESC 서보 객체 선언 추가

const uint8_t SERVO_PIN = 9;     // 핸들 제어용 D9
const uint8_t ESC_PIN   = 3;     // ESC 제어용 D3

const uint16_t MIN_PULSE = 800;
const uint16_t MAX_PULSE = 1800;

void setup() {
  Serial.begin(115200);
  steering.attach(SERVO_PIN);
  esc.attach(ESC_PIN);  // ✅ 선언한 esc 사용
}

void loop() {
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();  // 공백 제거

    if (s.length() > 0) {
      int commaIndex = s.indexOf(',');

      if (commaIndex > 0) {
        String s1 = s.substring(0, commaIndex);
        String s2 = s.substring(commaIndex + 1);

        int pulse   = s1.toInt();
        int pulse_2 = s2.toInt();

        pulse   = constrain(pulse,   MIN_PULSE, MAX_PULSE);
        pulse_2 = constrain(pulse_2, MIN_PULSE, MAX_PULSE);

        steering.writeMicroseconds(pulse);
        esc.writeMicroseconds(pulse_2);  // ✅ esc 사용

        Serial.print("Steering pulse: ");
        Serial.println(pulse);
        Serial.print("ESC pulse: ");
        Serial.println(pulse_2);
      }
    }
  }
}
