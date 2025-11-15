import pigpio

pi = pigpio.pi()
if pi.connected:
    print("? pigpio ?? ??!")
    pi.set_mode(18, pigpio.OUTPUT)
    pi.write(18, 1)
    pi.stop()
else:
    print("? pigpio ?? ?? ??. sudo pigpiod ????? ??!")
