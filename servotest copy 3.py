import RPi.GPIO as GPIO
import time

servo_pin = 12

GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)

pwm = GPIO.PWM(servo_pin, 50)  # 50Hz PWM
pwm.start(0)

try:
    duty_values = [  5.0, 5.5, 6.5,  7.5,  8.5,  9.5, 10.0]  # 0? ? 90? ? 180? ? 90?

    for duty in duty_values:
        print(f"Setting duty cycle: {duty}%")
        pwm.ChangeDutyCycle(duty)
        time.sleep(1)

except KeyboardInterrupt:
    pass

finally:
    pwm.stop()
    GPIO.cleanup()
