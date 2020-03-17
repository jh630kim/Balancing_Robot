# -*- coding: utf-8 -*-

# GPIO 라이브러리
import RPi.GPIO as GPIO
import time

'''
https://blog.naver.com/chandong83/221084688966를 참조
다만, wiringpi 이용을 RPi.GPIO를 이용하는 것으로 변경함
RPi.GPIO는 "라즈베리파이3 직접 코딩하기(앤써북)"을 참조
'''

# Mungchi의 예비 포트: X, VCC(12V), GND, 26, 19, 13, 6, 5, 11, VCC(3V)
# 실제 핀 정의
# PWM PIN
ENA = 13
IN1 = 19
IN2 = 26
IN3 = 16
IN4 = 20
ENB = 21

# 모터 채널
RIGHT = 0
LEFT = 1

# SW PWM 범위
PWM_FREQ = 100
pwm_R = ''
pwm_L = ''

def GPIO_init():
    global pwm_R, pwm_L

    # 경고 메시지 출력 방지
    GPIO.setwarnings(False)
    # GPIO Number에 따른 번호 부여
    GPIO.setmode(GPIO.BCM)
    # pwm 선언
    pwm_R = set_L298_pinconfig(ENA, IN1, IN2)
    pwm_L = set_L298_pinconfig(ENB, IN3, IN4)


def set_L298_pinconfig(EN, INA, INB):
    """
    :param EN: pin PWM
    :param INA: pin INA
    :param INB: pin INB
    :return: PWM handle
    # pin setup
    """
    GPIO.setup(INA, GPIO.OUT)
    GPIO.setup(INB, GPIO.OUT)
    GPIO.setup(EN, GPIO.OUT)
    # swPWM initialization
    pwm_handle = GPIO.PWM(EN, PWM_FREQ)
    pwm_handle.start(0)
    return pwm_handle


def set_L298_Control(PWM, INA, INB, speed):
    """ 모터 제어 함수 """
    # 앞으로
    if speed > 0:
        PWM.ChangeDutyCycle(speed)
        GPIO.output(INA, GPIO.LOW)
        GPIO.output(INB, GPIO.HIGH)

    # 뒤로
    elif speed < 0:
        PWM.ChangeDutyCycle(-speed)
        GPIO.output(INA, GPIO.HIGH)
        GPIO.output(INB, GPIO.LOW)

    # 정지
    elif speed == 0:
        PWM.ChangeDutyCycle(0)
        GPIO.output(INA, GPIO.LOW)
        GPIO.output(INB, GPIO.LOW)


def set_L298(ch, speed):
    """
    :param ch: RIGHT/LEFT, 오른족/왼쪽 모터 선택
    :param speed: 모터 속도(-PWM_FREQ(-100) ~ 0 ~ PWM_FREQ(100)), +: 전진, -: 후진, 0: 정지
    :return: 없음
    # 모터 제어함수 간단하게 사용하기 위해 한번더 래핑(감쌈)
    """
    if ch == RIGHT:
        set_L298_Control(pwm_R, IN1, IN2, speed)
    else:
        set_L298_Control(pwm_L, IN3, IN4, speed)


run = True     # 모터 구동 여부
debug = False    # 디버그 메시지 출력 여부

def motor_R(speed):
    """
    오른족 모터 구동
    :param speed: 모터 속도(-PWM_FREQ(-100) ~ 0 ~ PWM_FREQ(100)), +: 전진, -: 후진, 0: 정지
    """
    if run == True:
        set_L298(RIGHT, speed)

def motor_L(speed):
    """
    왼쪽 모터 구동
    :param speed:
    :param speed: 모터 속도(-PWM_FREQ(-100) ~ 0 ~ PWM_FREQ(100)), +: 전진, -: 후진, 0: 정지
    """
    if run == True:
        set_L298(LEFT, speed)


def motor(speed):
    """
    앙쪽 모터 구동
    :param speed: 모터 속도(-PWM_FREQ(-100) ~ 0 ~ PWM_FREQ(100)), +: 전진, -: 후진, 0: 정지
    """
    if run == True:
        set_L298(RIGHT, speed)
        set_L298(LEFT, speed)
    if debug == True:
        motor.count += 1
        if (motor.speed != speed) or (motor.count >= 100):
            motor.speed = speed
            motor.count = 0
            print("motor speed = ", speed)

# 함수의 속성(Attribute)을 이용한 static 변수 구현
motor.count = 0
motor.speed = 0


if __name__ == '__main__':
    ''' -----------------------------------'''
    ''' 모터 제어 테스트 '''
    ''' -----------------------------------'''
    # 1) GPIO 초기화
    GPIO_init()

    # 2) 제어 시작
    # R 전진 -> L 전진 -> 정지 -> L 후진 -> R 후진 -> 정지
    set_L298(RIGHT, 50)
    time.sleep(1)

    set_L298(LEFT, 50)
    time.sleep(1)

    set_L298(RIGHT, 0)
    set_L298(LEFT, 0)
    time.sleep(1)

    set_L298(LEFT, -50)
    time.sleep(1)

    set_L298(RIGHT, -50)
    time.sleep(1)

    set_L298(RIGHT, 0)
    set_L298(LEFT, 0)

    # 3) GPIO 종료
    GPIO.cleanup()
