# -*- coding: utf-8 -*-

import time

IO.setmode(IO.BCM)
IO.setwarnings(False)
IO.setup(encPinA, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(encPinB, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(pwmPin, IO.OUT)
IO.setup(dirPin, IO.OUT)

p = IO.PWM(19, 100)
p.start(0)

encoderPos = 0


def encoderA(channel):
    global encoderPos
    if IO.input(encPinA) == IO.input(encPinB):
        encoderPos += 1
    else:
        encoderPos -= 1


def encoderB(channel):
    global encoderPos
    if IO.input(encPinA) == IO.input(encPinB):
        encoderPos -= 1
    else:
        encoderPos += 1


IO.add_event_detect(encPinA, IO.BOTH, callback=encoderA)
IO.add_event_detect(encPinB, IO.BOTH, callback=encoderB)



​여기서부터는
PID가
들어간
핵심코드입니다.

PID에
대해서는
코드
밑에서
설명하겠습니다.

targetDeg = 360.
ratio = 360. / 270. / 64.
Kp = 1000.
Kd = 0.
Ki = 0.
dt = 0.
dt_sleep = 0.01
tolerance = 0.01

start_time = time.time()
error_prev = 0.
time_prev = 0.

while True:
    motorDeg = encoderPos * ratio

    error = targetDeg - motorDeg
    de = error - error_prev
    dt = time.time() - time_prev
    control = Kp * error + Kd * de / dt + Ki * error * dt;

    error_prev = error
    time_prev = time.time()

    IO.output(dirPin, control >= 0)
    p.ChangeDutyCycle(min(abs(control), 100))

    print('P-term = %7.1f, D-term = %7.1f, I-term = %7.1f' % (Kp * error, Kd * de / dt, Ki * de * dt))
    print('time = %6.3f, enc = %d, deg = %5.1f, err = %5.1f, ctrl = %7.1f' % (
    time.time() - start_time, encoderPos, motorDeg, error, control))
    print('%f, %f' % (de, dt))

    if abs(error) <= tolerance:
        IO.output(dirPin, control >= 0)
        p.ChangeDutyCycle(0)
        break

    time.sleep(dt_sleep)