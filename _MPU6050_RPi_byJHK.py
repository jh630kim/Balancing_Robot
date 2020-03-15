# -*- coding: utf-8 -*-

# GPIO 라이브러리
import smbus
import math
import time

'''
기본코드: http://practical.kr/?p=76
이론설명: https://blog.naver.com/codingbird/221766900497

참고:
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

# PWR_MGMT_1: sleep(bit 6)
SLEEP_EN        = 0x01 << 6
SLEEP_DIS       = 0x00 << 6
# PWR_MGMT_1: clock(bit 2:0)
CLOCK_INTERNAL  = 0x00  # internal clk(8KHz) 이용 (Not! Recommended)
CLOCK_PLL_XGYRO = 0x01  # XGyro와 동기
CLOCK_PLL_YGYRO = 0x02  # YGyro와 동기
CLOCK_PLL_ZGYRO = 0x03  # ZGyro와 동기

# Data 읽기
ACCEL_XOUT_H = 0x3B     # Low는 0x3C
ACCEL_YOUT_H = 0x3D     # Low는 0x3E
ACCEL_ZOUT_H = 0x3F     # Low는 0x40
GYRO_XOUT_H  = 0x43     # Low는 0x44
GYRO_YOUT_H  = 0x45     # Low는 0x46
GYRO_ZOUT_H  = 0x47     # Low는 0x48


# I2C Bus 초기화
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


def get_raw_data():
    """
    가속도(accel)와 각속도(gyro)의 현재 값 읽기
    :return: accel x/y/z, gyro x/y/z
    """
    gyro_xout = read_word_2c(GYRO_XOUT_H)
    gyro_yout = read_word_2c(GYRO_YOUT_H)
    gyro_zout = read_word_2c(GYRO_ZOUT_H)
    accel_xout = read_word_2c(ACCEL_XOUT_H)
    accel_yout = read_word_2c(ACCEL_YOUT_H)
    accel_zout = read_word_2c(ACCEL_ZOUT_H)
    return accel_xout, accel_yout, accel_zout,\
           gyro_xout, gyro_yout, gyro_zout


""" 
Accel값(가속도)만 이용해서 Y축과 X축 방향의 기울어진 각도 측정
중력가속도를 기준으로 기울여졌을때의 각 방향의 크기를 이용(삼각함수)
진동에 취약
"""
def cal_angle_acc(AcX, AcY, AcZ):
    """
    Accel값만 이용해서 X, Y의 각도 측정
    (고정 좌표 기준?)
    그런데... 각도가 0 -> 90 -> 0 -> -90 -> 0으로 바뀐다. 왜?
    0도 -> 90도 -> 180도 -> 270도 -> 360도
    즉, 30도와 120도가 모두 30도로 표시된다. 왜?
    :param AcX: Accel X
    :param AcY: Accel Y
    :param AcZ: Accel Z
    :return: X, Y angle in degree
    """
    y_radians = math.atan2(AcX, math.sqrt((AcY*AcY) + (AcZ*AcZ)))
    x_radians = math.atan2(AcY, math.sqrt((AcX*AcX) + (AcZ*AcZ)))
    return math.degrees(x_radians), -math.degrees(y_radians)


"""
자이로센서(각속도)를 이용한 각도 계산
각도 = 각속도 * 시간
오차가 누적됨
"""

DEGREE_PER_SECOND = 32767 / 250

past = 0      # 초
baseAcX = 0
baseAcY = 0
baseAcZ = 0
baseGyX = 0
baseGyY = 0
baseGyZ = 0

GyX_deg = 0
GyY_deg = 0
GyZ_deg = 0

def cal_angle_gyro(GyX, GyY, GyZ):
    """
    Gyro를 이용한 현재 각도 계산
    누적 방식이라... 회전하는 방향에 따라 양수/음수가 정해진다.
    :param y: 현재 Gyro 출력
    :return: 현재 각도, 기준 시간 -> past
    """
    global GyX_deg, GyY_deg, GyZ_deg

    now = time.time()
    dt = now - past     # 초단위

    GyX_deg += ((GyX - baseGyX) / DEGREE_PER_SECOND) * dt
    GyY_deg += ((GyY - baseGyY) / DEGREE_PER_SECOND) * dt
    GyZ_deg += ((GyZ - baseGyZ) / DEGREE_PER_SECOND) * dt

    return now      # 다음 계신을 위해 past로 저장되어야 한다.


def sensor_calibration():
    """
    1초동안의 평균을 이용하여 기준점 계산
    :return: Accel과 Gyro의 기준점 -> baseAcX ~ basGyZ
    """
    SumAcX = 0
    SumAcY = 0
    SumAcZ = 0
    SumGyX = 0
    SumGyY = 0
    SumGyZ = 0

    for i in range(10):
        AcX, AcY, AcZ, GyX, GyY, GyZ = get_raw_data()
        SumAcX += AcX
        SumAcY += AcY
        SumAcZ += AcZ
        SumGyX += GyX
        SumGyY += GyY
        SumGyZ += GyZ
        time.sleep(0.1)

    avgAcX = SumAcX / 10
    avgAcY = SumAcY / 10
    avgAcZ = SumAcZ / 10
    avgGyX = SumGyX / 10
    avgGyY = SumGyY / 10
    avgGyZ = SumGyZ / 10

    return avgAcX, avgAcY, avgAcZ, avgGyX, avgGyY, avgGyZ



# Now wake the 6050 up as it starts in sleep mode
def set_MPU6050_init(dlpf_bw=DLPF_BW_256,
                     gyro_fs=GYRO_FS_250, accel_fs=ACCEL_FS_2,
                     clk_pll=CLOCK_PLL_XGYRO):
    global baseAcX, baseAcY, baseAcZ, baseGyX, baseGyY, baseGyZ, past

    # MPU6050 초기값 setting
    write_byte(PWR_MGMT_1, SLEEP_EN | clk_pll)      # sleep mode(bit6), clock(bit2:0)은 XGyro 동기
    write_byte(CONFIG, dlpf_bw)                     # bit 2:0
    write_byte(GYRO_CONFIG, gyro_fs)                # Gyro Full Scale bit 4:3
    write_byte(ACCEL_CONFIG, accel_fs)              # Accel Full Scale Bit 4:3
    write_byte(PWR_MGMT_1, SLEEP_DIS | clk_pll)     # Start

    # sensor 계산 초기화
    baseAcX, baseAcY, baseAcZ, baseGyX, baseGyY, baseGyZ \
        = sensor_calibration()
    past = time.time()

    return read_byte(PWR_MGMT_1)


'''
Sensor Read
# GPIO Pin
FRONT = 11
REAR  = 12

def SENSOR_init():
    GPIO.setup(FRONT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(REAR, GPIO.IN, pull_up_down=GPIO.PUD_UP)
'''

if __name__ == '__main__':
    ''' -----------------------------------'''
    ''' Gyro 테스트 '''
    ''' -----------------------------------'''
    test = set_MPU6050_init(dlpf_bw=DLPF_BW_98)
    print("Gyro PWR_MGMT_1 Register = ", test)


    sensor_calibration()
    cnt = 0
    while True:
        AcX, AcY, AcZ, GyX, GyY, GyZ = get_raw_data()

        AcX_deg, AcY_deg = cal_angle_acc(AcX, AcY, AcZ)
        past = cal_angle_gyro(GyX, GyY, GyZ)

        # print("AcX_deg, AcY_deg = ", AcX_deg, ',', AcY_deg)
        time.sleep(0.01)
        cnt += 1
        if cnt%100 == 0:
            print("GyX,Y,Z_deg = ", GyX_deg, ',', GyY_deg, ',', GyZ_deg)



