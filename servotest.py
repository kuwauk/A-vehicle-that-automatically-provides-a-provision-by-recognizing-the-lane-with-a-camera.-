import pigpio
import sys

# 핀 설정 (BCM 번호)
SERVO_PIN = 9   # 핸들 서보
ESC_PIN   = 3   # ESC 제어

# PWM 범위
MIN_PULSE = 800
MAX_PULSE = 1800

# pigpio 초기화
pi = pigpio.pi()
if not pi.connected:
    print("pigpio 데몬 실행 필요: sudo pigpiod")
    sys.exit(1)

# 핀을 출력으로 설정
pi.set_mode(SERVO_PIN, pigpio.OUTPUT)
pi.set_mode(ESC_PIN, pigpio.OUTPUT)

print("Ready. Input format: pulse1,pulse2")

try:
    while True:
        s = input().strip()
        if "," in s:
            s1, s2 = s.split(",")
            try:
                pulse   = int(s1)
                pulse_2 = int(s2)

                # 범위 제한
                pulse   = max(MIN_PULSE, min(MAX_PULSE, pulse))
                pulse_2 = max(MIN_PULSE, min(MAX_PULSE, pulse_2))

                # 서보 및 ESC 제어
                pi.set_servo_pulsewidth(SERVO_PIN, pulse)
                pi.set_servo_pulsewidth(ESC_PIN, pulse_2)

                print(f"Steering pulse: {pulse}")
                print(f"ESC pulse: {pulse_2}")

            except ValueError:
                print("⚠️ 숫자 입력 필요")
except KeyboardInterrupt:
    print("\n종료 중...")

# 종료시 서보 off
pi.set_servo_pulsewidth(SERVO_PIN, 0)
pi.set_servo_pulsewidth(ESC_PIN, 0)
pi.stop()
