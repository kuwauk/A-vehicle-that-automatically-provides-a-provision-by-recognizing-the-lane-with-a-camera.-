import RPi.GPIO as GPIO
import time

# Servo control pin
_SERVO_PIN = 12       # BCM numbering
_FREQUENCY = 50       # 40Hz PWM

GPIO.setmode(GPIO.BCM)
GPIO.setup(_SERVO_PIN, GPIO.OUT)

_pwm = GPIO.PWM(_SERVO_PIN, _FREQUENCY)
_pwm.start(7.5)

_prev_duty = 7.5      # Initial duty (1.5ms / 20ms * 100)

def set_pulse_us(pulse):
    global _prev_duty             # ? MUST be first statement

    """
    pulse: pulse width in microseconds (e.g. 800..1600)
    """
    # 1) Convert �s pulse to % duty over 20ms (20000�s)
    duty = pulse / 20000 * 100

    # 2) Only update if change ? 0.2%
    if abs(duty - _prev_duty) >= 0.2:
        print(f"[Servo] Pulse={pulse}�s -> Duty={duty:.1f}%")
        _pwm.ChangeDutyCycle(duty)
        _prev_duty = duty      # now safe to reassign
        # 3) short stabilization delay
        time.sleep(0.1)
