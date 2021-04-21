#!/usr/bin/env python3
'''This program attempts to legally navigate the course and lock a target.

It is a fundamental behavior program that:
 - Implements the RGB, Pixy, and US sensors
 - Actuates the motors
 - Attempts to navigate the course legally (but not necessarily logically)
'''
import sys
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
        active_cmd = 'x'
        STATE = 'TRAVELLING'

        with busio.I2C(board.SCL, board.SDA) as bus:
            mux = adafruit_tca9548a.TCA9548A(bus)
            rgb_left = adafruit_tcs34725.TCS34725(mux[7])
            rgb_right = adafruit_tcs34725.TCS34725(mux[5])
            motor = I2CDevice(mux[0], MOTOR_CTRL_ADDR, probe=False)

            with motor:
                motor.write_then_readinto(motor_lib.FORWARD_CMD, read_buff)
            active_cmd = chr(read_buff[0])

            while 1:
                #read rgbs
                lux_l = rgb_left.lux
                lux_r = rgb_right.lux
                left_edge = lux_l>700
                right_edge = lux_r>700
                # if loop_count%200 == 0:
                #     print(f'Lux L:{lux_l:.1f}, R:{lux_r:.1f}')
                #     print(f'State: {STATE}')

                #apply motor logic
                if left_edge and right_edge:
                    #Stop CMD
                    new_cmd = motor_lib.STOP_CMD
                    if active_cmd != new_cmd[0]:
                        with motor:
                            motor.write_then_readinto(new_cmd, read_buff)
                        active_cmd = chr(read_buff[0])
                    STATE = 'DONE'

                if STATE == 'TRAVELLING':
                    if not left_edge and not right_edge:
                        pass
                    else:
                        #Stop CMD
                        new_cmd = motor_lib.STOP_CMD
                        if active_cmd != new_cmd[0]:
                            with motor:
                                motor.write_then_readinto(new_cmd, read_buff)
                            active_cmd = chr(read_buff[0])
                        STATE = 'BACKING'

                elif STATE == 'BACKING':
                    if left_edge:
                        #Diag over BR burst
                        new_cmd = motor_lib.DIAG_BR_CMD
                        if active_cmd != new_cmd[0]:
                            with motor:
                                motor.write_then_readinto(new_cmd, read_buff)
                            active_cmd = chr(read_buff[0])
                        time.sleep(0.5)
                        #Rot about BL until edge event
                        new_cmd = motor_lib.ROT_BL_CMD
                        if active_cmd != new_cmd[0]:
                            with motor:
                                motor.write_then_readinto(new_cmd, read_buff)
                            active_cmd = chr(read_buff[0])
                        STATE = 'CHECK_LEFT'
                    elif right_edge:
                        #diag over BL burst
                        new_cmd = motor_lib.DIAG_BL_CMD
                        if active_cmd != new_cmd[0]:
                            with motor:
                                motor.write_then_readinto(new_cmd, read_buff)
                            active_cmd = chr(read_buff[0])
                        time.sleep(0.5)
                        #Rot about BR until edge event
                        new_cmd = motor_lib.ROT_BR_CMD
                        if active_cmd != new_cmd[0]:
                            with motor:
                                motor.write_then_readinto(new_cmd, read_buff)
                            active_cmd = chr(read_buff[0])
                        STATE = 'CHECK_RIGHT'
                    else:
                        STATE = 'ERROR'
                elif STATE == 'CHECK_LEFT':
                    if left_edge:
                        #stop cmd
                        new_cmd = motor_lib.STOP_CMD
                        if active_cmd != new_cmd[0]:
                            with motor:
                                motor.write_then_readinto(new_cmd, read_buff)
                            active_cmd = chr(read_buff[0])
                        STATE = 'BACKING'
                    elif right_edge:
                        #stop cmd
                        new_cmd = motor_lib.STOP_CMD
                        if active_cmd != new_cmd[0]:
                            with motor:
                                motor.write_then_readinto(new_cmd, read_buff)
                            active_cmd = chr(read_buff[0])
                        STATE = 'ALIGNING'
                elif STATE == 'CHECK_RIGHT':
                    if left_edge:
                        #stop cmd
                        new_cmd = motor_lib.STOP_CMD
                        if active_cmd != new_cmd[0]:
                            with motor:
                                motor.write_then_readinto(new_cmd, read_buff)
                            active_cmd = chr(read_buff[0])
                        STATE = 'ALIGNING'
                    elif right_edge:
                        #stop cmd
                        new_cmd = motor_lib.STOP_CMD
                        if active_cmd != new_cmd[0]:
                            with motor:
                                motor.write_then_readinto(new_cmd, read_buff)
                            active_cmd = chr(read_buff[0])
                        STATE = 'BACKING'
                elif STATE == 'ALIGNING':
                    if left_edge:
                        #Rot L
                        new_cmd = motor_lib.ROT_L_CMD
                        if active_cmd != new_cmd[0]:
                            with motor:
                                motor.write_then_readinto(new_cmd, read_buff)
                            active_cmd = chr(read_buff[0])
                    elif right_edge:
                        #Rot R
                        new_cmd = motor_lib.ROT_R_CMD
                        if active_cmd != new_cmd[0]:
                            with motor:
                                motor.write_then_readinto(new_cmd, read_buff)
                            active_cmd = chr(read_buff[0])
                    else:
                        #forward
                        new_cmd = motor_lib.FORWARD_CMD
                        if active_cmd != new_cmd[0]:
                            with motor:
                                motor.write_then_readinto(new_cmd, read_buff)
                            active_cmd = chr(read_buff[0])
                elif STATE == 'DONE':
                    print('Mission accomplished!')
                    sys.exit()
                else:
                    print('!Bad State Logic')
                    time.sleep(30)

                if STATE == 'ERROR':
                    #stop cmd
                    new_cmd = motor_lib.STOP_CMD
                    if active_cmd != new_cmd[0]:
                        with motor:
                            motor.write_then_readinto(new_cmd, read_buff)
                        active_cmd = chr(read_buff[0])
                    print('!Hit Error State')
                    time.sleep(30)

                # print(f'{STATE}')
                time.sleep(0.02)
                loop_count += 1
    except KeyboardInterrupt:
        with motor:
            motor.write_then_readinto(motor_lib.STOP_CMD, read_buff)
        print('exited gracefully')
