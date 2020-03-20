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
import datetime

from simple_pid import PID


########################################
# PID 초기화
########################################
# todo: 로봇에 맞게 튜닝 필요
# Config파일에서 PID 설정값 읽기
file = open("_01_config.py", 'r')
setpoint = float(file.readline().split("#")[0])       # 로봇이 지면에서 평형을 유지하는 상태의 값
Kp = float(file.readline().split("#")[0])             # P gain, 1단계 설정
Ki = float(file.readline().split("#")[0])            # I gain, 3단계 설정
Kd = float(file.readline().split("#")[0])            # D gain, 2단계 설정
run = int(file.readline().split("#")[0])            # 모터 구동 여부
logging = int(file.readline().split("#")[0])        # data logging 여부
file.close()

# 파일 실행 인자로 설정값 받기
try:
    if sys.argv[1] != 'r':
        setpoint = float(sys.argv[1])
except IndexError:
    print("Set (1)setpoint, (1)Kp, (3)Ki, (4)Kd, (5)motor_run[1], (6)logging[0] status!")
    print("('-' for not changing)")
    print("Or, Use [_00_Balancing.py r] for defalut setting")
    exit()

# 인자로 - 를 받으면 default값 이용
try:
    if sys.argv[2] != '-':
        Kp = float(sys.argv[2])
    if sys.argv[3] != '-':
        Ki = float(sys.argv[3])
    if sys.argv[4] != '-':
        Kd = float(sys.argv[4])
    if sys.argv[5] != '-':
        run = int(sys.argv[5])
    if sys.argv[6] != '-':
        logging = int(sys.argv[6])
except IndexError:
    pass

# Config파일에 PID 설정값 쓰기
file = open("_01_config.py", 'w')
file.write("{}      # set_point\n".format(setpoint))
file.write("{}      # P gain\n".format(Kp))
file.write("{}      # I gain\n".format(Ki))
file.write("{}      # D gain\n".format(Kd))
file.write("{}      # motor on = 1, off = 0\n".format(run))
file.write("{}      # data logging on `= 1, off = 0\n".format(logging))
file.close()

# 기타 설정값
sample_time = 0.01  # 주기: 0.05 sec
output_limits = (-100, 100)   # 출력 제한: -100 ~ 100
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

# PID 설정값 Logging
dt = datetime.datetime.now()
dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second, dt.microsecond

file_name = './_01_run_setting_log.txt'
file=open(file_name, 'a')
file.write('{}/{} {}:{} {} {} {} {} {}\n'.format(
    dt.month, dt.day, dt.hour, dt.minute, setpoint, Kp, Ki, Kd, sample_time))
file.close()

########################################
# MOU-6050 초기화
########################################
# Sensor initialization
mpu6050 = Angle_MPU6050()
mpu6050.start_measure_thread()

########################################
# L298(모터) 초기화
########################################
L298.GPIO_init()

########################################
# Client Socket 초기화
########################################
# 전송 센서 정보
if logging == 1:
    sensor_name = ["Kalman_Y", "DMP_Z", "Gyro_Z", "MOTOR"]

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
print("Kp = {}, Ki = {}, Kd = {}".format(Kp, Ki, Kd))

'''
for i in range(3, 0, -1):
    print(i, "!")
    time.sleep(1)

print("Start!!!")
'''
# 초기값 설정
count = 0           # display 주기
past = time.time()  # 현재 시간(Loop time 확인용)

while True:
    # 센서 읽기
    gyro_yaw = mpu6050.get_gyro_yaw()
    kalman_pitch = mpu6050.get_kalman_pitch()
    DMP_yaw = mpu6050.get_DMP_yaw()

    # 현재 각도
    current_angle = kalman_pitch

    # PID 제어
    # current_angle = 현재 Y 각도(pitch)
    # motor_speed: 모터의 전/후진
    motor_speed = pid(current_angle)
    if run == 1:
        # todo: 모터 출력이 선형적인지 확인
        #       스텝모터면 편한데... 다른 프로젝트에서는 Encoder로 회전수를 확인하기도 했다.
        L298.motor(motor_speed)   # 그런데... 힘이 약한 것 같다. 출력이 선형이 아닌것도 같고...

        # scipia의 모터 제어 방식
        '''
        if motor_speed > 0:
            L298.motor(FORWARD)
        elif motor_speed < 0:
            L298.motor(BACKWARD)
        else:
            L298.motor(STOP)
        '''

    # 메시지 생성 for logging
    data.append(value.format(kalman_pitch, DMP_yaw, gyro_yaw, motor_speed))

    count += 1
    # todo: PID에서 Sampling Rate에 따라 기다리는지 확인
    #       현재 Sampling Rate는 0.01임
    # time.sleep(0.01)

    # 100개 메시지가 쌓이면, 그래프를 그리기 위해 전송(또는 Logging)
    if count >= 100:
        new = time.time()

        rec.msg_send(msg, data)
        
        print("Elapsed time(100) =", new - past)
        past = new

        # 메시지 전송 후 초기화
        data = []
        count = 0

# GPIO 종료
GPIO.cleanup()

# 소켓 닫기
rec.close()

