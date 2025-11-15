import RPi.GPIO as GPIO
import time

ESC_GPIO = 13  # ??? GPIO ? ??
GPIO.setmode(GPIO.BCM)
GPIO.setup(ESC_GPIO, GPIO.OUT)

pwm = GPIO.PWM(ESC_GPIO, 50)  # 50Hz PWM ??
pwm.start(0)

def set_duty_cycle(pulse_width_us):
    duty_cycle = (pulse_width_us / 20000) * 100
    pwm.ChangeDutyCycle(duty_cycle)

try:
    # ESC ???: ?? ??? 2?? ??
    set_duty_cycle(1500)
    time.sleep(2)

    # ??? ???
    set_duty_cycle(1000)
    time.sleep(10)

    # ??? ???
    set_duty_cycle(2000)
    time.sleep(10)

    # ???? ??
    set_duty_cycle(1500)
    time.sleep(2)

    # ?? ??
    set_duty_cycle(0)
    time.sleep(2)

finally:
    pwm.stop()
    GPIO.cleanup()
