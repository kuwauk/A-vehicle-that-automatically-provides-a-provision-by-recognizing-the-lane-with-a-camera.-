import RPi.GPIO as GPIO
import time

pin = 12
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin, GPIO.OUT)

pwm = GPIO.PWM(pin, 40)  # 50Hz for servo
pwm.start(0)

def set_pulse_us(pulse):
    duty = pulse / 20000 * 100
    pwm.ChangeDutyCycle(duty)
    print(f"Pulse: {pulse} �s -> Duty: {duty:.2f}%")

try:
    for pulse in range(800, 1601, 50):
        set_pulse_us(pulse)
        time.sleep(0.05)
except KeyboardInterrupt:
    pass
finally:
    pwm.stop()
    GPIO.cleanup()
