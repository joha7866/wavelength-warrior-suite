#!/usr/bin/env python3
'''This modules implements the Adafruit LSM6DSOX IMU.

Key behaviors to accomplish are:
- Turn as exactly an input angle as possible.
- Mark a start and integrate gyro over time to evaluate angle of arbitrary turn.
- Mark and start and double integrate accelerometer over time to evaluate distance of arbitrary start/stop.

Current Debug Functionality:
- Setup mux, motor driver, IMU
- Run 90, 180, 360 turns for visual verification.
'''
import time
import math
import board
import busio
import adafruit_tca9548a
from adafruit_lsm6ds import lsm6dsox
from adafruit_bus_device.i2c_device import I2CDevice

import motor_smbus as motor_lib

def Travel(distance):
    distance = distance/100
    with busio.I2C(board.SCL, board.SDA) as bus:
        mux = adafruit_tca9548a.TCA9548A(bus)
        imu = lsm6dsox.LSM6DSOX(mux[2])
        motor = I2CDevice(mux[0], motor_lib.MOTOR_CTRL_ADDR, probe=False)
        read_buff = bytearray(16)
        with motor:
            motor.write_then_readinto(motor_lib.FORWARD_CMD, read_buff)

        displacement = 0
        velocity = 0
        t1 = time.time()
        while abs(displacement)<(abs(distance)+math.sqrt(distance*2.5)):
            t2 = time.time()
            d_vel = math.sqrt(imu.acceleration[0]**2 * imu.acceleration[1]**2)
            velocity += d_vel*(t2-t1)
            displacement += velocity*(t2-t1)
            t1 = t2
        with motor:
                motor.write_then_readinto(motor_lib.STOP_CMD, read_buff)

def eval_angle(imu, motor, input_angle):
    '''Rotate exactly a given angle
    imu: imu object
    motor: motor ctrl object
    input_angle: input angle in rad/s
    '''
    read_buff = bytearray(16)
    if input_angle < 0:
         motor.send_cmd(motor_lib.ROT_R_CMD)
    else:
         motor.send_cmd(motor_lib.ROT_L_CMD)

    angle_z = 0
    t1 = time.time()
    while abs(angle_z)<abs(input_angle)-0.1:
        t2 = time.time()
        d_theta = imu.gyro[2]
        angle_z += d_theta*(t2-t1)
        t1 = t2
    motor.send_cmd(motor_lib.STOP_CMD)
    return angle_z

def R90():
    with busio.I2C(board.SCL, board.SDA) as bus:
        mux = adafruit_tca9548a.TCA9548A(bus)
        imu = lsm6dsox.LSM6DSOX(mux[2])
        motor = I2CDevice(mux[0], motor_lib.MOTOR_CTRL_ADDR, probe=False)
        read_buff = bytearray(16)
        with motor:
            motor.write_then_readinto(motor_lib.ROT_R_CMD, read_buff)

        angle_z = 0
        t1 = time.time()
        while abs(angle_z)<abs(math.pi/2)-0.05:
            t2 = time.time()
            d_theta = imu.gyro[2]
            angle_z += d_theta*(t2-t1)
            t1 = t2
        with motor:
                motor.write_then_readinto(motor_lib.STOP_CMD, read_buff)

def L90():
    with busio.I2C(board.SCL, board.SDA) as bus:
        mux = adafruit_tca9548a.TCA9548A(bus)
        imu = lsm6dsox.LSM6DSOX(mux[2])
        motor = I2CDevice(mux[0], motor_lib.MOTOR_CTRL_ADDR, probe=False)
        read_buff = bytearray(16)
        with motor:
            motor.write_then_readinto(motor_lib.ROT_L_CMD, read_buff)

        angle_z = 0
        t1 = time.time()
        while abs(angle_z)<abs(math.pi/2)-0.05:
            t2 = time.time()
            d_theta = imu.gyro[2]
            angle_z += d_theta*(t2-t1)
            t1 = t2
        with motor:
                motor.write_then_readinto(motor_lib.STOP_CMD, read_buff)

def T180():
    with busio.I2C(board.SCL, board.SDA) as bus:
        mux = adafruit_tca9548a.TCA9548A(bus)
        imu = lsm6dsox.LSM6DSOX(mux[2])
        motor = I2CDevice(mux[0], motor_lib.MOTOR_CTRL_ADDR, probe=False)
        read_buff = bytearray(16)
        with motor:
            motor.write_then_readinto(motor_lib.ROT_R_CMD, read_buff)

        angle_z = 0
        t1 = time.time()
        while abs(angle_z)<abs(math.pi)-0.065:
            t2 = time.time()
            d_theta = imu.gyro[2]
            angle_z += d_theta*(t2-t1)
            t1 = t2
        with motor:
                motor.write_then_readinto(motor_lib.STOP_CMD, read_buff)
            
def T360():
    with busio.I2C(board.SCL, board.SDA) as bus:
        mux = adafruit_tca9548a.TCA9548A(bus)
        imu = lsm6dsox.LSM6DSOX(mux[2])
        motor = I2CDevice(mux[0], motor_lib.MOTOR_CTRL_ADDR, probe=False)
        read_buff = bytearray(16)
        with motor:
            motor.write_then_readinto(motor_lib.ROT_L_CMD, read_buff)

        angle_z = 0
        t1 = time.time()
        while abs(angle_z)<abs(math.pi*2)-0.15:
            t2 = time.time()
            d_theta = imu.gyro[2]
            angle_z += d_theta*(t2-t1)
            t1 = t2
        with motor:
                motor.write_then_readinto(motor_lib.STOP_CMD, read_buff)

if __name__ == "__main__":
    with busio.I2C(board.SCL, board.SDA) as bus:
        mux = adafruit_tca9548a.TCA9548A(bus)
        imu = lsm6dsox.LSM6DSOX(mux[2])
        motor = I2CDevice(mux[0], motor_lib.MOTOR_CTRL_ADDR, probe=False)

        try:
            R90()
            print("Finished right turn!")
            L90()
            print("Finished left turn!")
            T180()
            print("Turned 180!")
            # Travel(40)
            T360()
            print("Turned 360!")

        except KeyboardInterrupt:
            with motor:
                motor.write_then_readinto(motor_lib.STOP_CMD, read_buff)
            print('exited gracefully')
        

        #     read_buff = bytearray(16)
            
        #     last_angle = eval_angle(imu, motor, -(math.pi/2))
        #     return_angle = last_angle*-1
        #     eval_angle(imu,motor,return_angle)
