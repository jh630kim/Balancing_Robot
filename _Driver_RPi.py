# -*- coding: utf-8 -*-

# GPIO 라이브러리
import RPi.GPIO as GPIO
import smbus
import math
import time

'''
https://blog.naver.com/chandong83/221084688966를 참조
다만, wiringpi 이용을 RPi.GPIO를 이용하는 것으로 변경함
RPi.GPIO는 "라즈베리파이3 직접 코딩하기(앤써북)"을 참조
'''

# X, VCC(12V), GND, 26, 19, 13, 6, 5, 11, VCC(5V or 3V)
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


def GPIO_init():
    # GPIO Number에 따른 번호 부여
    GPIO.setmode(GPIO.BCM)
    # 경고 메시지 출력 방지
    GPIO.setwarnings(False)


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
        GPIO.output(INA, GPIO.HIGH)
        GPIO.output(INB, GPIO.LOW)

    # 뒤로
    elif speed < 0:
        PWM.ChangeDutyCycle(-speed)
        GPIO.output(INA, GPIO.LOW)
        GPIO.output(INB, GPIO.HIGH)

    # 정지
    elif speed < 0:
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

'''
http://practical.kr/?p=76를 참조

sudo apt-get install i2c-tools      # i2cdetect 사용하기 위한 툴 설치
i2cdetect -l                        # i2c 모듈 확인
i2cdetect -y 1                      # i2c 연결 확인 (MPU-6050은 0x68번) 
i2cdetect -y 1 0x68                 # 0x68의 256바이트 읽기
i2cdetect -y 1 0x68 0x75            # 0x68의 0x75 레지스터 값 읽기

sudo apt-get install python-smbus   # smbus 설치
'''

# MPU-6050 Register
CONFIG       = 0x1A     # LowPassFilter bit 2:0
GYRO_CONFIG  = 0x1B     # FS_SEL bit 4:3
ACCEL_CONFIG = 0x1C     # FS_SEL bit 4:3
PWR_MGMT_1   = 0x6B     # sleep bit 6, clk_select bit 2:0

# CONFIG: Low Pass Filter 설정(bit 2:0)
DLPF_BW_256 = 0x00      # Acc: BW-260Hz, Delay-0ms, Gyro: BW-256Hz, Delay-0.98ms
DLPF_BW_188 = 0x01
DLPF_BW_98  = 0x02
DLPF_BW_42  = 0x03
DLPF_BW_20  = 0x04
DLPF_BW_10  = 0x05
DLPF_BW_5   = 0x06      # Acc: BW-5Hz, Delay-19ms, Gyro: BW-5Hz, Delay-18.6ms

# GYRO_CONFIG: Gyro의 Full Scale 설정(bit 4:3)
GYRO_FS_250  = 0x00 << 3    # 250 deg/sec
GYRO_FS_500  = 0x01 << 3
GYRO_FS_1000 = 0x02 << 3
GYRO_FS_2000 = 0x03 << 3    # 2000 deg/sec

# ACCEL_CONFIG: 가속도센서의 Full Scale 설정(bit 4:3)
ACCEL_FS_2  = 0x00 << 3     # 2g
ACCEL_FS_4  = 0x01 << 3
ACCEL_FS_8  = 0x02 << 3
ACCEL_FS_16 = 0x03 << 3     # 16g

# PWR_MGMT_1: sleep(bit6)
SLEEP_EN        = 0x01 << 6
SLEEP_DIS       = 0x00 << 6
# PWR_MGMT_1; clock(bit 2:0)
CLOCK_INTERNAL  = 0x00  # internal clk(8KHz) 이용
CLOCK_PLL_XGYRO = 0x01  # XGyro와 동기 (Recommended)
CLOCK_PLL_YGYRO = 0x02  # YGyro와 동기
CLOCK_PLL_ZGYRO = 0x03  # ZGyro와 동기

# Data 읽기
ACCEL_XOUT_H = 0x3B     # Low는 0x3C
ACCEL_YOUT_H = 0x3D     # Low는 0x3E
ACCEL_ZOUT_H = 0x3F     # Low는 0x40
TEMP_OUT_H   = 0x41     # Low는 0x42
GYRO_XOUT_H  = 0x43     # Low는 0x44
GYRO_YOUT_H  = 0x45     # Low는 0x46
GYRO_ZOUT_H  = 0x47     # Low는 0x48


# I2C 버스 및 Address 선안
I2C_bus = smbus.SMBus(1)
MPU6050_addr = 0x68


def write_byte(adr, data):
    I2C_bus.write_byte_data(MPU6050_addr, adr, data)


def read_byte(adr):
    return I2C_bus.read_byte_data(MPU6050_addr, adr)


def read_word(adr):
    high = I2C_bus.read_byte_data(MPU6050_addr, adr)
    low = I2C_bus.read_byte_data(MPU6050_addr, adr+1)
    val = (high << 8) + low
    return val


def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val


# Now wake the 6050 up as it starts in sleep mode
def set_MPU6050_init(dlpf_bw=DLPF_BW_256,
                     gyro_fs=GYRO_FS_250, accel_fs=ACCEL_FS_2,
                     clk_pll=CLOCK_PLL_XGYRO):
    write_byte(PWR_MGMT_1, SLEEP_EN | clk_pll)      # sleep mode(bit6), clock(bit2:0)은 XGyro 동기
    write_byte(CONFIG, dlpf_bw)                     # bit 2:0
    write_byte(GYRO_CONFIG, gyro_fs)                # Gyro Full Scale bit 4:3
    write_byte(ACCEL_CONFIG, accel_fs)              # Accel Full Scale Bit 4:3
    write_byte(PWR_MGMT_1, SLEEP_DIS | clk_pll)     # Start

    return read_byte(PWR_MGMT_1)


'''
Sensor Read
'''
# GPIO Pin
FRONT = 11
REAR  = 12

def SENSOR_init():
    GPIO.setup(FRONT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(REAR, GPIO.IN, pull_up_down=GPIO.PUD_UP)


if __name__ == '__main__':

    GPIO_init()
    ''' -----------------------------------'''
    ''' GPIO SENSOR 테스트 '''
    ''' -----------------------------------'''
    '''
    SENSOR_init()
    while True:
        front = GPIO.input(FRONT)
        rear = GPIO.input(REAR)
        print("Front:", front, " Rear:", rear)
        if (front == 1) && (rear == 1):
            break
    '''

    ''' -----------------------------------'''
    ''' 모터 제어 테스트 '''
    ''' -----------------------------------'''
    # 설정
    pwm_R = set_L298_pinconfig(ENA, IN1, IN2)
    pwm_L = set_L298_pinconfig(ENB, IN3, IN4)

    # 제어 시작
    # R 전진 -> L 전진 -> 정지 -> L 후진 -> R 후진 -> 정지
    pwm_R.start(10)
    pwm_L.start(10)
    time.sleep(1)
    '''
    set_L298(RIGHT, 10)
    time.sleep(1)

    set_L298(LEFT, 10)
    time.sleep(1)

    set_L298(RIGHT, 0)
    set_L298(LEFT, 0)
    time.sleep(1)

    set_L298(LEFT, -10)
    time.sleep(1)
    set_L298(RIGHT, -10)
    time.sleep(1)

    set_L298(RIGHT, 0)
    set_L298(LEFT, 0)
    '''
    GPIO.cleanup()

    ''' -----------------------------------'''
    ''' Gyro 테스트 '''
    ''' -----------------------------------'''
    test = set_MPU6050_init()
    print("Gyro PWR_MGMT_1 Register = ", test)


    while True:
        gyro_xout = read_word_2c(GYRO_XOUT_H)
        gyro_yout = read_word_2c(GYRO_YOUT_H)
        gyro_zout = read_word_2c(GYRO_ZOUT_H)

        accel_xout = read_word_2c(ACCEL_XOUT_H)
        accel_yout = read_word_2c(ACCEL_YOUT_H)
        accel_zout = read_word_2c(ACCEL_ZOUT_H)

        print("gyro_X/Y/Z: ", gyro_xout, "/", gyro_yout, "/", gyro_zout,
              ", Accel X/Y/Z: ", accel_xout, "/", accel_yout, "/", accel_zout)
        time.sleep(1)
