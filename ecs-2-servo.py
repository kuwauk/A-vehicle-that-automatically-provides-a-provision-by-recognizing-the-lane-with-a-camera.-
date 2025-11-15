import RPi.GPIO as GPIO
import time

_SERVO_PIN = 12
_FREQUENCY = 50

GPIO.setmode(GPIO.BCM)
GPIO.setup(_SERVO_PIN, GPIO.OUT)

_pwm = GPIO.PWM(_SERVO_PIN, _FREQUENCY)
_pwm.start(0)

def us_to_duty(pulse_us: int) -> float:
    return pulse_us / 20000 * 100

try:
    pulse_values = range(1100, 1801, 100)  # 1100?s ~ 1800?s, 1?s ?? ??
    for pulse in pulse_values:
        duty = us_to_duty(pulse)
        print(f"Setting pulse: {pulse}?s ? duty: {duty:.3f}%")
        _pwm.ChangeDutyCycle(duty)
        time.sleep(0.5)

except KeyboardInterrupt:
    pass

finally:
    if _pwm is not None:
        _pwm.ChangeDutyCycle(0)
        _pwm.stop()
    GPIO.cleanup()

