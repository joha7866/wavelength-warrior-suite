#!/usr/bin/env python3
'''This program attempts to legally navigate the course and lock a target.

It is a fundamental behavior program that:
 - Implements the RGB, Pixy, and US sensors
 - Actuates the motors
 - Attempts to navigate the course legally (but not necessarily logically)
'''
import time
import board
import busio
import adafruit_tca9548a
import adafruit_tcs34725
import adafruit_hcsr04
from adafruit_bus_device.i2c_device import I2CDevice

import modules.motor_smbus as motor_lib

#i2c addressses
MOTOR_CTRL_ADDR = 0x69
PIXY_ADDR = 0x54
MAX_I2C_MSG_BYTES = 16

#gpio definitions
LEFT_ULTRASONIC_PAIR = [19,26]
RIGHT_ULTRASONIC_PAIR = [16,20]

if __name__ == "__main__":
    try:
        read_buff = bytearray(16)
        loop_count = 0
        STATE = 'TRAVELLING'

        with busio.I2C(board.SCL, board.SDA) as bus:
            mux = adafruit_tca9548a.TCA9548A(bus)
            rgb_left = adafruit_tcs34725.TCS34725(mux[5])
            rgb_right = adafruit_tcs34725.TCS34725(mux[7])
            motor = I2CDevice(mux[0], MOTOR_CTRL_ADDR, probe=False)

            while 1:
                #read rgbs
                lux_l = rgb_left.lux
                lux_r = rgb_right.lux
                left_edge = lux_l>800
                right_edge = lux_r>800
                # if loop_count%10 == 0:
                #     print(f'Lux L:{lux_l:.1f}, R:{lux_r:.1f}')

                #apply motor logic
                if left_edge and right_edge:
                    with motor:
                        motor.write_then_readinto(motor_lib.STOP_CMD, read_buff)
                    STATE = 'DONE'
                if STATE == 'TRAVELLING':
                    if not left_edge and not right_edge:
                        with motor:
                            motor.write_then_readinto(motor_lib.FORWARD_CMD, read_buff)
                    else:
                        with motor:
                            motor.write_then_readinto(motor_lib.STOP_CMD, read_buff)
                        STATE = 'BACKING'
                elif STATE == 'BACKING':
                    if left_edge:
                        #Diag over BR burst
                        with motor:
                            motor.write_then_readinto(motor_lib.DIAG_BR_CMD, read_buff)
                        time.sleep(0.5)
                        #Rot about BL until edge event
                        with motor:
                            motor.write_then_readinto(motor_lib.ROT_BL_CMD, read_buff)
                        STATE = 'CHECKING_LEFT'
                    elif right_edge:
                        #diag over BL burst
                        with motor:
                            motor.write_then_readinto(motor_lib.DIAG_BL_CMD, read_buff)
                        time.sleep(0.5)
                        #Rot about BR until edge event
                        with motor:
                            motor.write_then_readinto(motor_lib.ROT_BR_CMD, read_buff)
                        STATE = 'CHECKING_RIGHT'
                    else:
                        STATE = 'ERROR'
                elif STATE == 'CHECKING_LEFT':
                    if left_edge:
                        with motor:
                            motor.write_then_readinto(motor_lib.STOP_CMD, read_buff)
                        STATE = 'BACKING'
                    elif right_edge:
                        with motor:
                            motor.write_then_readinto(motor_lib.STOP_CMD, read_buff)
                        STATE = 'ALIGNING'
                elif STATE == 'CHECKING_RIGHT':
                    if left_edge:
                        with motor:
                            motor.write_then_readinto(motor_lib.STOP_CMD, read_buff)
                        STATE = 'ALIGNING'
                    elif right_edge:
                        with motor:
                            motor.write_then_readinto(motor_lib.STOP_CMD, read_buff)
                        STATE = 'BACKING'
                elif STATE == 'ALIGNING':
                    if left_edge:
                        with motor:
                            motor.write_then_readinto(motor_lib.ROT_L_CMD, read_buff)
                    elif right_edge:
                        with motor:
                            motor.write_then_readinto(motor_lib.ROT_R_CMD, read_buff)
                    else:
                        with motor:
                            motor.write_then_readinto(motor_lib.FORWARD_CMD, read_buff)
                elif STATE == 'DONE':
                    print('Mission accomplished!')
                    time.sleep(30)
                else:
                    print('!Bad State Logic')
                    time.sleep(30)

                if STATE == 'ERROR':
                    with motor:
                            motor.write_then_readinto(motor_lib.STOP_CMD, read_buff)
                    print('!Hit Error State')
                    time.sleep(30)

                loop_count += 1
    except KeyboardInterrupt:
        with motor:
            motor.write_then_readinto(motor_lib.STOP_CMD, read_buff)
        print('exited gracefully')

