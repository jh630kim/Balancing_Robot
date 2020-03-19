# -*- coding: utf-8 -*-

"""
# 참고: http://scipia.co.kr/cms/blog/227
# 아두이노 코드를 라즈베리파이의 python 코드로 수정
# PID는 아래 코드 이용
# https://pypi.org/project/simple-pid/
"""

import _10_L298_RPi as L298
from _21_MPU_Thread import Angle_MPU6050
# Sensor_Visualizer_v2를 이용해 Display하고 싶으면 이걸 이용
# import _41_LAN_Comm as rec
# 단순히 파일로 기록하고 싶으면 이걸 이용
import _41_FILE_record as rec

import math
import time
import sys

from simple_pid import PID


########################################
# MOU-6050 초기화
########################################
# Sensor initialization
mpu6050 = Angle_MPU6050()
mpu6050.start_measure_thread()

########################################
# PID 초기화
########################################
# todo: 로봇에 맞게 튜닝 필요
setpoint = 3.5       # 로봇이 지면에서 평형을 유지하는 상태의 값
Kp = 10             # P gain, 1단계 설정
Kd = 0              # D gain, 2단계 설정
Ki = 0              # I gain, 3단계 설정

try:
    Kp = float(sys.argv[1])
    Ki = float(sys.argv[2])
    Kd = float(sys.argv[3])
except IndexError:
    pass

print("Kp = {}, Ki = {}, Kd = {}".format(Kp, Ki, Kd))
# 0, 25, 0, 5
# 10, 25, 0, 5
# 기타 설정값
sample_time = 0.05  # 주기: 0.05 sec
    # 제어 주기를 10ms로 하고, sensor 주기를 200Hz로 하면
    # 현재 python 성능 상 한번 roop를 도는 데 0.033sec가 걸린다
    # 그리고, 계속 FIFO overflow가 난다.
    # 즉, 그만한 성능을 내지 못하는거다.
    # 따라서, 제어 주기를 50ms로 변경하고, sensor 주기는 125Hz로 변경
output_limits = (-95, 95)   # 출력 제한: -50 ~ 50
auto_mode = True    # PID control ON(True), Off(False)
p_ON_M = False      # 뭐지?

# PID 제어기 선언
pid = PID(Kp=Kp, Ki=Ki, Kd=Kd,
          setpoint=setpoint,
          sample_time=sample_time,
          output_limits=output_limits,
          auto_mode=auto_mode,
          proportional_on_measurement=p_ON_M)
# 참고: 설정값 변경 예
# pid.setpoint = 10
# pid.tunings = (1.0, 0,2, 0.4) or pid.Kp = 1.0

########################################
# L298(모터) 초기화
########################################
L298.GPIO_init()

FORWARD = 95
BACKWARD = -95
STOP = 0

########################################
# Client Socket 초기화
########################################
# 전송 센서 정보
sensor_name = ["Kalman_Y", "Comp_Y", "Accel_Y", "DMP_Y",
               "Kalman_X", "Comp_X", "Accel_X", "DMP_X",
               "Gyro_X", "Gyro_Y", "Gyro_Z",
               "MOTOR"]

channel = len(sensor_name)

# Data 메시지 준비
msg = 'value'
data = []
value = ''
sensor_min = []
sensor_max = []

for i in range(channel):
    value += '{} '
    sensor_min.append(-180)
    sensor_max.append(180)

# Clinet 초기화
rec.initialize(channel, sensor_name, sensor_min, sensor_max)

########################################
# 제어 루프 시작
########################################
# 초기값 설정
count = 0           # display 주기
past = time.time()  # 현재 시간(Loop time 확인용)

while True:
    # 읽을 센서값이 있는지 확인(Int Status와 fifo count를 확인)
    accel_roll = mpu6050.get_accel_roll()
    accel_pitch = mpu6050.get_accel_pitch()
    gyro_roll = mpu6050.get_gyro_roll()
    gyro_pitch = mpu6050.get_gyro_pitch()
    gyro_yaw = mpu6050.get_gyro_yaw()
    compl_roll = mpu6050.get_complementary_roll()
    compl_pitch = mpu6050.get_complementary_pitch()
    kalman_roll = mpu6050.get_kalman_roll()
    kalman_pitch = mpu6050.get_kalman_pitch()
    DMP_roll = mpu6050.get_DMP_roll()
    DMP_pitch = mpu6050.get_DMP_pitch()
    DMP_yaw = mpu6050.get_DMP_yaw()

    # current_angle = kalman_pitch
    current_angle = DMP_pitch

    # PID 제어
    # current_angle = 현재 Y 각도(pitch)
    # motor_speed: 모터의 전/후진
    motor_speed = pid(current_angle)
    L298.motor(motor_speed)   # 그런데... 힘이 약한 것 같다. 출력이 선형이 아닌것도 같고...
    # print('{0:5.2f}, {0:5.2f}'.format(DMP_pitch, y_degree))
    
    # scipia의 코딩 방식
    '''
    if motor_speed > 0:
        L298.motor(FORWARD)
    elif motor_speed < 0:
        L298.motor(BACKWARD)
    else:
        L298.motor(STOP)
    '''

    # 메시지 생성
    data.append(value.format(kalman_pitch, compl_pitch, accel_pitch, DMP_pitch,
                             kalman_roll, compl_roll, accel_roll, DMP_roll,
                             gyro_roll, gyro_pitch, gyro_yaw,
                             motor_speed))

    count += 1
    time.sleep(0.01)

    # 100개 메시지가 쌓이면, 그래프를 그리기 위해 전송
    if count >= 100:
        new = time.time()

        rec.msg_send(msg, data)
        
        print("Elapsed time(100) =", new - past)
        past = new
        # print('in: %5.5f, out: %+4.2f, current_accel[2]: %f, current_gyro[2]: %f' %
        #       (current_angle, motor_speed, current_accel[2], current_gyro[2]))
        # print(msg)
        # 메시지 전송 후 초기화
        msg = 'value'
        data = []
        count = 0

# GPIO 종료
GPIO.cleanup()

# 소켓 닫기
rec.close()

