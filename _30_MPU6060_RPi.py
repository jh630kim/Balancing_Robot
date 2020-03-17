# -*- coding: utf-8 -*-

import time
import math
import _31_mpu6050 as mpu6050

# Sensor initialization
mpu = mpu6050.MPU6050()
mpu.dmpInitialize()
mpu.setDMPEnabled(True)

# get expected DMP packet size for later comparison
packetSize = mpu.dmpGetFIFOPacketSize()
count = 0
past = time.time()

sample_rate = mpu.getRate()
dlp_mode = mpu.getDLPFMode()
gyro_fs = mpu.getFullScaleGyroRange()
accel_fs = mpu.getFullScaleAccelRange()
print('sample_rate = ', sample_rate,
      ', dlp_mode = ', dlp_mode,
      ', gyro_fs = ', gyro_fs,
      ', accel_fs = ', accel_fs)

while True:
    # Get INT_STATUS byte
    mpuIntStatus = mpu.getIntStatus()

    if mpuIntStatus >= 2: # check for DMP data ready interrupt (this should happen frequently)
        # get current FIFO count
        fifoCount = mpu.getFIFOCount()

        # check for overflow (this should never happen unless our code is too inefficient)
        # if fifoCount == 1024:
        if (fifoCount == 1024) or (mpuIntStatus & 0x10):
            # reset so we can continue cleanly
            mpu.resetFIFO()
            # print('FIFO overflow!')
            print('overflow!', fifoCount, mpuIntStatus & 0x10)

        # wait for correct available data length, should be a VERY short wait
        fifoCount = mpu.getFIFOCount()
        while fifoCount < packetSize:
            fifoCount = mpu.getFIFOCount()

        result = mpu.getFIFOBytes(packetSize)
        q = mpu.dmpGetQuaternion(result)
        g = mpu.dmpGetGravity(q)
        ypr = mpu.dmpGetYawPitchRoll(q, g)

        new = time.time()
        # print(ypr['yaw'] * 180 / math.pi),
        # print(ypr['pitch'] * 180 / math.pi),
        # print(ypr['roll'] * 180 / math.pi)
        if count % 10 == 0:
            print(ypr['yaw'] * 180 / math.pi, ',',
                  ypr['pitch'] * 180 / math.pi, ',',
                  ypr['roll'] * 180 / math.pi, ',',
                  new - past)

        # track FIFO count here in case there is > 1 packet available
        # (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize

        past = new
        count += 1