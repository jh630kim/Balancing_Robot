# -*- coding: utf-8 -*-

"""
copy from https://github.com/thisisG/MPU6050-I2C-Python-Class
__author__ = 'Geir Istad'
"""

from _21_MPU6050 import MPU6050

# I2C 버스 초기화
i2c_bus = 1
device_address = 0x68

# The offsets are different for each device and should be changed
# accordingly using a calibration procedure
# todo: Offset을 바꾸라는데... 어떻게 바꾸라는 건지...
# MPU6050의 set_x_gyro_offset 부분의 주석을 바꿔야 한다.
x_accel_offset = 0
y_accel_offset = 0
z_accel_offset = 0
x_gyro_offset = 0
y_gyro_offset = 0
z_gyro_offset = 0
enable_debug_output = True

# MPU Instance 생성
# cpp 클래스에는 offset이 없었는데...
mpu = MPU6050(i2c_bus, device_address,
              x_accel_offset, y_accel_offset, z_accel_offset,
              x_gyro_offset, y_gyro_offset, z_gyro_offset,
              enable_debug_output)

mpu.dmp_initialize()
mpu.set_DMP_enabled(True)
mpu_int_status = mpu.get_int_status()
print('mpu int status = ', hex(mpu_int_status))

packet_size = mpu.DMP_get_FIFO_packet_size()
print('packet size = ', packet_size)
FIFO_count = mpu.get_FIFO_count()
print('FIFO count = ', FIFO_count)

count = 0
FIFO_buffer = [0]*64

FIFO_count_list = list()
while count < 10000:
    FIFO_count = mpu.get_FIFO_count()
    mpu_int_status = mpu.get_int_status()

    # If overflow is detected by status or fifo count we want to reset
    if (FIFO_count == 1024) or (mpu_int_status & 0x10):
        mpu.reset_FIFO()
        print('overflow!', FIFO_count, mpu_int_status&0x10)
    # Check if fifo data is ready
    elif (mpu_int_status & 0x02):
        # Wait until packet_size number of bytes are ready for reading, default
        # is 42 bytes
        while FIFO_count < packet_size:
            FIFO_count = mpu.get_FIFO_count()

        FIFO_buffer = mpu.get_FIFO_bytes(packet_size)
        accel = mpu.DMP_get_acceleration_int16(FIFO_buffer)
        quat = mpu.DMP_get_quaternion_int16(FIFO_buffer)
        grav = mpu.DMP_get_gravity(quat)
        roll_pitch_yaw = mpu.DMP_get_euler_roll_pitch_yaw(quat, grav)
        if count % 10 == 0:
            print(roll_pitch_yaw.x, ',',
                  roll_pitch_yaw.y, ',',
                  roll_pitch_yaw.z)
        count += 1
