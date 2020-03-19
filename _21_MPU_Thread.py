# -*- coding: utf-8 -*-

# Copy From https://github.com/rocheparadox/Kalman-Filter-Python-for-mpu6050

# Connections
# MPU6050 - Raspberry pi
# VCC - 5V  (2 or 4 Board)
# GND - GND (6 - Board)
# SCL - SCL (5 - Board)
# SDA - SDA (3 - Board)

from _51_Kalman import KalmanAngle
import smbus  # import SMBus module of I2C
import time
import math
import threading
import _31_mpu6050 as mpu6050


class Angle_MPU6050:
    # Read the gyro and acceleromater values from MPU6050
    def __init__(self):
        self.pitch = 0
        self.roll = 0
        self.accel_roll = 0
        self.accel_pitch = 0
        self.gyro_roll = 0
        self.gyro_pitch = 0
        self.gyro_yaw = 0
        self.compl_roll = 0
        self.compl_pitch = 0
        self.kalman_roll = 0
        self.kalman_pitch = 0
        self.DMP_roll = 0
        self.DMP_pitch = 0
        self.DMP_yaw = 0
        self.mpu = 0
        self.packetSize = 0

        self.MPU_Init()


    # Thread 시작
    def start_measure_thread(self):
        angleThread1 = threading.Thread(target=self.measureAngles)
        angleThread2 = threading.Thread(target=self.measureAngles_DMP)
        angleThread1.start()
        angleThread2.start()


    # MPU-6050 초기화
    def MPU_Init(self):
        # Sensor initialization
        self.mpu = mpu6050.MPU6050()
        self.mpu.dmpInitialize()
        self.mpu.setDMPEnabled(True)

        # get expected DMP packet size for later comparison
        self.packetSize = self.mpu.dmpGetFIFOPacketSize()

        # Kalman Filter 예제의 추가 설정
        # Write to power management register
        self.mpu.setClockSource(3)   # Kalman = 1(X), DMP = 3(Z)

        # Setting DLPF (last three bit of 0X1A to 6 i.e '110'
        # It removes the noise due to vibration.)
        # Kalman = BW_5(18.6ms Delay), DMP = BW_42(4.8ms Delay)
        # self.mpu.setDLPFMode(self.mpu.MPU6050_DLPF_BW_5)

        # Write to interrupt enable register
        # DMP: 0x12 - FIFO and DMP int enable
        # Kalman: 0x01 - Data_Rdy int enable
        # DMP + Kalman = 0x13
        self.mpu.setIntEnabled(0x13)


    # Kalman 필터 계산 쓰레드
    def measureAngles(self):
        flag = 0
        kalmanX = KalmanAngle()
        kalmanY = KalmanAngle()

        RestrictPitch = True  
        # Comment out to restrict roll to ±90deg instead - 
        # please read: 
        # http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
        radToDeg = 57.2957786
        kalAngleX = 0
        kalAngleY = 0

        time.sleep(1)
        # Read Accelerometer raw value
        accel_XYZ = self.mpu.getAcceleration()
        accX = accel_XYZ[0]
        accY = accel_XYZ[1]
        accZ = accel_XYZ[2]

        # print(accX,accY,accZ)
        # print(math.sqrt((accY**2)+(accZ**2)))
        # todo: 이건 뭐지?
        #       accel을 이용한 roll pitch도 아니고...
        #       y_radians = math.atan2(AcX, math.sqrt((AcY*AcY) + (AcZ*AcZ)))
        #       x_radians = math.atan2(AcY, math.sqrt((AcX*AcX) + (AcZ*AcZ)))
        #       roll = math.degrees(x_radians), pitch = -math.degree(y_radians)
        if (RestrictPitch):
            roll = math.degrees(math.atan2(accY, accZ))
            pitch = math.degrees(math.atan(-accX / math.sqrt((accY ** 2) + (accZ ** 2))))
        else:
            roll = math.atan(accY / math.sqrt((accX ** 2) + (accZ ** 2))) * radToDeg
            pitch = math.atan2(-accX, accZ) * radToDeg
        # print(roll)
        kalmanX.setAngle(roll)
        kalmanY.setAngle(pitch)
        gyroXAngle = roll;
        gyroYAngle = pitch;
        compAngleX = roll;
        compAngleY = pitch;

        timer = time.time()
        flag = 0

        while True:

            if (flag > 100):  # Problem with the connection
                print("There is a problem with the connection")
                flag = 0
                continue

            try:
                ##############################################################
                # Kalman Filter와 상보(Complementary) Filter에 의한 각도 계산
                ##############################################################
                # 1) 현재 Accel과 Gyro 계산
                # Read Accelerometer raw value
                accel_XYZ = self.mpu.getAcceleration()
                accX = accel_XYZ[0]
                accY = accel_XYZ[1]
                accZ = accel_XYZ[2]

                # Read Gyroscope raw value
                gyro_XYZ = self.mpu.getRotation()
                gyroX = gyro_XYZ[0]
                gyroY = gyro_XYZ[1]
                gyroZ = gyro_XYZ[2]

                # 2) 소요 시간 측정
                dt = time.time() - timer
                timer = time.time()

                #3) Accel을 이용한 Roll/Pitch 계산 같은데...
                # todo: 이건 뭐지?
                #       RestrictPitch
                if (RestrictPitch):
                    roll = math.degrees(math.atan2(accY, accZ))
                    pitch = math.degrees(math.atan(-accX / math.sqrt((accY ** 2) + (accZ ** 2))))
                else:
                    roll = math.atan(accY / math.sqrt((accX ** 2) + (accZ ** 2))) * radToDeg
                    pitch = math.atan2(-accX, accZ) * radToDeg

                # 3-1) Accel을 이용한 roll/pitch
                self.accel_roll = math.degrees(math.atan2(accX, math.sqrt((accY * accY) + (accZ * accZ))))
                self.accel_pitch = math.degrees(math.atan2(accY, math.sqrt((accX*accX) + (accZ*accZ))))

                # by JHK
                # todo: to be checked
                #       131은 Gyro의 FS이 0일때의 Scale Factor인데...
                #       현재 Gyro의 FS은 3이다.
                #       이때의 FS은 16.4!!! 그런데... Datasheet를 믿을수가 없다.
                # gyroXRate = gyroX / 131
                # gyroYRate = gyroY / 131
                gyroXRate = gyroX / 16.4
                gyroYRate = gyroY / 16.4
                gyroZRate = gyroZ / 16.4

                if (RestrictPitch):
                    if ((roll < -90 and kalAngleX > 90) or (roll > 90 and kalAngleX < -90)):
                        kalmanX.setAngle(roll)
                        complAngleX = roll
                        kalAngleX = roll
                        gyroXAngle = roll
                    else:
                        kalAngleX = kalmanX.getAngle(roll, gyroXRate, dt)

                    # 뭐야 이건...
                    if (abs(kalAngleY) > 90 or True):
                        gyroYRate = -gyroYRate
                        kalAngleY = kalmanY.getAngle(pitch, gyroYRate, dt)
                else:
                    if ((pitch < -90 and kalAngleY > 90) or (pitch > 90 and kalAngleY < -90)):
                        kalmanY.setAngle(pitch)
                        complAngleY = pitch
                        kalAngleY = pitch
                        gyroYAngle = pitch
                    else:
                        kalAngleY = kalmanY.getAngle(pitch, gyroYRate, dt)

                    if (abs(kalAngleX) > 90):
                        gyroXRate = -gyroXRate
                        kalAngleX = kalmanX.getAngle(roll, gyroXRate, dt)

                # 3-2) Gyro를 이용한 roll/pitch
                # angle = (rate of change of angle) * change in time
                gyroXAngle = gyroXRate * dt
                gyroYAngle = gyroYRate * dt
                gyroZangle = gyroZRate * dt

                self.gyro_roll = gyroXAngle
                self.gyro_pitch = gyroYAngle
                self.gyro_yaw = gyroZangle

                # 3-3) 상보필터로 계산하는 roll/pitch
                # 계수는 0.93...
                # compAngle = constant * (old_compAngle + angle_obtained_from_gyro) + constant * angle_obtained from accelerometer
                compAngleX = 0.93 * (compAngleX + gyroXRate * dt) + 0.07 * roll
                compAngleY = 0.93 * (compAngleY + gyroYRate * dt) + 0.07 * pitch

                if ((gyroXAngle < -180) or (gyroXAngle > 180)):
                    gyroXAngle = kalAngleX
                if ((gyroYAngle < -180) or (gyroYAngle > 180)):
                    gyroYAngle = kalAngleY

                # print("Angle X: " + str(complAngleX)+"   " +"Angle Y: " + str(complAngleY))
                self.pitch = compAngleY
                self.roll = compAngleX

                self.kalman_pitch = kalAngleY
                self.kalman_roll = kalAngleX
                self.compl_pitch = compAngleY
                self.compl_roll = compAngleX
                time.sleep(0.005)

            except Exception as exc:
                if (flag == 100):
                    print(exc)
                flag += 1


    # DMP 계산 Thread
    def measureAngles_DMP(self):
        while True:
            # 읽을 센서값이 있는지 확인(Int Status와 fifo count를 확인)
            mpuIntStatus = self.mpu.getIntStatus()
            fifoCount = self.mpu.getFIFOCount()

            # Overflow 확인
            # this should never happen unless our code is too inefficient
            if (fifoCount == 1024) or (mpuIntStatus & 0x10):
                # reset so we can continue cleanly
                self.mpu.resetFIFO()
                print('FIFO overflow!', fifoCount, hex(mpuIntStatus))

            # 읽을 데이터가 있을 때 까지 대기
            fifoCount = self.mpu.getFIFOCount()
            while fifoCount < self.packetSize:
                fifoCount = self.mpu.getFIFOCount()

            # 수신 데이터가 있는 경우
            result = self.mpu.getFIFOBytes(self.packetSize)
            quaternion = self.mpu.dmpGetQuaternion(result)
            gravity = self.mpu.dmpGetGravity(quaternion)
            ypr = self.mpu.dmpGetYawPitchRoll(quaternion, gravity)

            # DMP를 이용해 계산한 각도
            self.DMP_roll = math.degrees(ypr['roll'])
            self.DMP_pitch = math.degrees(ypr['pitch'])
            self.DMP_yaw = math.degrees(ypr['yaw'])

            time.sleep(0.005)

    # 외부에서 센서값 읽기
    # 센서의 설치 위치에 따라 불연속적으롤 보일 수있다.
    # 즉, -179 -> +179로 갈 수 있다.
    # 센서 사용 범위에 따라 적절히 조절해야 한다.
    def get_roll(self):
        return self.roll

    def get_pitch(self):
        return self.pitch

    def get_complementary_roll(self):
        # by JHK: DMP, accel과 맞추기 위해 조정
        # return self.compl_roll
        return -self.compl_roll

    def get_complementary_pitch(self):
        # by JHK: DMP, accel과 맞추기 위해 조정
        # return self.compl_pitch
        return -self.compl_pitch

    def get_kalman_roll(self):
        # by JHK: DMP, accel과 맞추기 위해 조정
        # return self.kalman_roll
        '''
        self.kalman_roll = -self.kalman_roll
        if self.kalman_roll < 0:
            self.kalman_roll += 360
        self.kalman_roll -= 180
        '''
        return -self.kalman_roll

    def get_kalman_pitch(self):
        # by JHK: DMP, accel과 맞추기 위해 조정
        # return self.kalman_pitch
        return -self.kalman_pitch

    def get_DMP_roll(self):
        return self.DMP_roll

    def get_DMP_pitch(self):
        return self.DMP_pitch

    def get_DMP_yaw(self):
        return self.DMP_yaw

    def get_accel_roll(self):
        return self.accel_roll

    def get_accel_pitch(self):
        return self.accel_pitch

    def get_gyro_roll(self):
        return self.gyro_roll

    def get_gyro_pitch(self):
        return self.gyro_pitch

    def get_gyro_yaw(self):
        return self.gyro_yaw
