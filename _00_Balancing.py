# -*- coding: utf-8 -*-

"""
# 참고: http://scipia.co.kr/cms/blog/227
# 아두이노 코드를 라즈베리파이의 python 코드로 수정
# PID는 아래 코드 이용
# https://pypi.org/project/simple-pid/
"""

import _10_L298_RPi as L298
import _31_mpu6050 as mpu6050
# Sensor_Visualizer_v2를 이용해 Display하고 싶으면 이걸 이용
# import _41_LAN_Comm as rec
# 단순히 파일로 기록하고 싶으면 이걸 이용
import _41_FILE_record as rec
import math
import time

from simple_pid import PID


########################################
# MOU-6050 초기화
########################################
# Sensor initialization
mpu = mpu6050.MPU6050()
mpu.dmpInitialize()
mpu.setDMPEnabled(True)

# get expected DMP packet size for later comparison
packetSize = mpu.dmpGetFIFOPacketSize()

# DMP 설정 확인
sample_rate = mpu.getRate()
dlp_mode = mpu.getDLPFMode()
gyro_fs = mpu.getFullScaleGyroRange()
accel_fs = mpu.getFullScaleAccelRange()
print('sample_rate = ', sample_rate,
      ', dlp_mode = ', dlp_mode,
      ', gyro_fs = ', gyro_fs,
      ', accel_fs = ', accel_fs)

'''
# Quaternion q; // [w, x, y, z] quaternion container 
# VectorFloat gravity; // [x, y, z] gravity vector
# float ypr[3]; // [yaw, pitch, roll] yaw / pitch / roll container and gravity vector
'''

########################################
# PID 초기화
########################################
# todo: 로봇에 맞게 튜닝 필요
setpoint = 4.5      # 로봇이 지면에서 평형을 유지하는 상태의 값
Kp = 30             # P gain, 1단계 설정
Kd = 2              # D gain, 2단계 설정
Ki = 5              # I gain, 3단계 설정
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
channel = 9
blk = ' '
sensor_name = ["ROLL(X)", "PITCH(Y)", "YAW(Z)",
               "ACC(X)", "ACC(Y)", "ACC(Z)",
               "DEG", "SPEED", "GYRO(Z)"]
sensor_min = [-180, -180, -180, -30000, -30000, -30000, -180, -100, -30000]
sensor_max = [ 180,  180,  180,  30000,  30000,  30000,  180,  100,  30000]

# Clinet 초기화
rec.initialize(channel, sensor_name, sensor_min, sensor_max)

# Data 메시지 준비
msg = 'value'
data = []
value = '{} {} {} {} {} {} {} {} {}'

########################################
# 제어 루프 시작
########################################
# 초기값 설정
count = 0           # display 주기
past = time.time()  # 현재 시간(Loop time 확인용)

while True:
    # 읽을 센서값이 있는지 확인(Int Status와 fifo count를 확인)
    mpuIntStatus = mpu.getIntStatus()
    fifoCount = mpu.getFIFOCount()

    # Overflow 확인
    # this should never happen unless our code is too inefficient
    if (fifoCount == 1024) or (mpuIntStatus & 0x10):
        # reset so we can continue cleanly
        mpu.resetFIFO()
        print('FIFO overflow!', fifoCount, hex(mpuIntStatus))

    # 읽을 데이터가 있을 때 까지 대기
    fifoCount = mpu.getFIFOCount()
    while fifoCount < packetSize:
        fifoCount = mpu.getFIFOCount()

    # 수신 데이터가 있는 경우
    result = mpu.getFIFOBytes(packetSize)
    quaternion = mpu.dmpGetQuaternion(result)
    gravity = mpu.dmpGetGravity(quaternion)
    ypr = mpu.dmpGetYawPitchRoll(quaternion, gravity)

    # "FIFO buffer"에서 읽은 각도
    current_roll = ypr['roll'] * 180 / math.pi
    current_pitch = ypr['pitch'] * 180 / math.pi
    current_yaw = ypr['yaw'] * 180 / math.pi
    
    # "현재" accel(가속도)와 gyro(각속도) 읽기
    # [0]: X, [1]: Y, [2]: Z
    current_accel = mpu.getAcceleration()
    current_gyro = mpu.getRotation()
    
    AcX = current_accel[0]
    AcY = current_accel[1]
    AcZ = current_accel[2]
    
    y_radians = math.atan2(AcX, math.sqrt((AcY*AcY) + (AcZ*AcZ)))
    x_radians = math.atan2(AcY, math.sqrt((AcX*AcX) + (AcZ*AcZ)))
    
    y_degree = y_radians * 180 / math.pi
    x_degree = x_radians * 180 / math.pi

    # 최종 입력값(전/후 기울기)
    current_angle = current_pitch
    # current_angle = y_degree

    # PID 제어
    # current_angle = 현재 Y 각도(pitch)
    # motor_speed: 모터의 전/후진
    motor_speed = pid(current_angle)
    L298.motor(motor_speed)   # 그런데... 힘이 약한 것 같다. 출력이 선형이 아닌것도 같고...
    # print('{0:5.2f}, {0:5.2f}'.format(current_pitch, y_degree))
    
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
    data.append(value.format(current_roll, current_pitch, current_yaw,
                             current_accel[0], current_accel[1], current_accel[2],
                             current_pitch, y_degree, motor_speed))

    count += 1

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

