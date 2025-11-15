# ecsmotor.py
import RPi.GPIO as GPIO

# Only one ESC control pin (GPIO 13)
_ECS_PIN = 13
_FREQUENCY = 50  # 50Hz PWM

GPIO.setmode(GPIO.BCM)
GPIO.setup(_ECS_PIN, GPIO.OUT)
_pwm = GPIO.PWM(_ECS_PIN, _FREQUENCY)
_pwm.start()

def set_pwm(left: float, right: float):
    """
    Set PWM duty cycle for ESC based on speed.
    Only one motor pin is used.

    Args:
        left (float): -1.0..1.0 (used as input signal)
        right (float): ignored (placeholder for compatibility)
    """
    speed = (left + right) / 2
    def map_dc(x):
        return max(5.0, min(10.0, 7.5 + (x * 2.5)))
    _pwm.ChangeDutyCycle(map_dc(speed))
