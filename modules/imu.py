import time
import math
import board
import busio
import adafruit_tca9548a
from adafruit_lsm6ds import lsm6dsox
from adafruit_bus_device.i2c_device import I2CDevice

import modules.motor_smbus as motor_lib

def eval_angle(imu, motor, input_angle):
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

if __name__ == "__main__":
    with busio.I2C(board.SCL, board.SDA) as bus:
        mux = adafruit_tca9548a.TCA9548A(bus)
        imu = lsm6dsox.LSM6DSOX(mux[2])
        motor = I2CDevice(mux[0], motor_lib.MOTOR_CTRL_ADDR, probe=False)

        read_buff = bytearray(16)
        
        last_angle = eval_angle(imu, motor, -(math.pi/2))
        return_angle = last_angle*-1
        eval_angle(imu,motor,return_angle)


