import RPi.GPIO as GPIO
import time

servo_pin = 12

GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)

pwm = GPIO.PWM(servo_pin, 50)  # 50Hz
pwm.start(0)

def rotate(direction="stop"):
    if direction == "forward":
        pwm.ChangeDutyCycle(6)  # ??? ??
    elif direction == "backward":
        pwm.ChangeDutyCycle(9)  # ??? ??
    else:
        pwm.ChangeDutyCycle(7.5)  # ??

try:
    rotate("forward")
    time.sleep(2)

    rotate("backward")
    time.sleep(2)

    rotate("stop")
    time.sleep(1)

finally:
    pwm.stop()
    GPIO.cleanup()
