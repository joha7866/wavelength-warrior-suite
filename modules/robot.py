#!/usr/bin/env python3
'''This module'''
import time
import sys
import math
import board
import busio
import adafruit_tca9548a
import adafruit_tcs34725
import adafruit_hcsr04
from adafruit_lsm6ds import lsm6dsox
from adafruit_bus_device.i2c_device import I2CDevice

import modules.motor_smbus as motor_lib
import modules.imu as imu_lib
import modules.pixy_smbus as pixy_lib

class Robot(object):
    def __init__(self, bus):
        self.bus = bus

        self.mux = adafruit_tca9548a.TCA9548A(bus)
        self.imu = lsm6dsox.LSM6DSOX(self.mux[2])
        self.rgb_left = adafruit_tcs34725.TCS34725(self.mux[7])
        self.rgb_right = adafruit_tcs34725.TCS34725(self.mux[5])
        self.motor = motor_lib.MotorController(self.mux[0])
        self.pixy = I2CDevice(self.mux[4], pixy_lib.PIXY_ADDR)
    
    def do_turn(self, dir='left', timeout=10):
        '''test'''

        if dir=='left':
            angle = math.pi/2
        else:
            angle = -math.pi/2

        #burst back
        self.motor.send_cmd(motor_lib.BACKWARD_CMD)
        time.sleep(0.5)
        self.motor.send_cmd(motor_lib.STOP_CMD)

        imu_lib.eval_angle(self.imu, self.motor, angle)


    def stop(self):
        ''''''
        self.motor.send_cmd(motor_lib.STOP_CMD)

    def cross_purples(self, count=2):
        self.motor.send_cmd(motor_lib.FORWARD_CMD)
        hit_count = 0
        while 1:
            #read rgbs
            rl, gl, bl, cl = self.rgb_left.color_raw
            rr, gr, br, cr = self.rgb_right.color_raw

            left_purple = 50>=cl>17 and 25>=rl>8
            right_purple = 50>=cr>17 and 25>=rr>8
            left_yellow = cr>50 and rl>25
            right_yellow = cr>50 and rr>25

            if left_yellow or right_yellow:
                self.motor.send_cmd(motor_lib.STOP_CMD)
                print('stopped on edge')
                return -1

            if left_purple or right_purple:
                hit_count += 1
                if hit_count >= count:
                    self.motor.send_cmd(motor_lib.STOP_CMD)
                    print('thru')
                    return 0
                time.sleep(0.4)


    def do_align(self, timeout=10):
        '''Classified align behavior.
        Return 1 if success 0 if timeout.
        '''
        state = 'TRAVELLING'
        read_buff = bytearray(16)
        loop_count = 0
        start_ts = time.time()

        self.motor.send_cmd(motor_lib.FORWARD_CMD)

        while time.time() < start_ts+timeout:
            #read rgbs
            lux_l = self.rgb_left.lux
            lux_r = self.rgb_right.lux
            left_edge = lux_l>700
            right_edge = lux_r>700

            #apply motor logic
            if left_edge and right_edge:
                self.motor.send_cmd(motor_lib.STOP_CMD)
                state = 'DONE'

            if state == 'TRAVELLING':
                if not left_edge and not right_edge:
                    pass
                else:
                    self.motor.send_cmd(motor_lib.STOP_CMD)
                    state = 'BACKING'

            elif state == 'BACKING':
                if left_edge:
                    #Diag over BR burst
                    self.motor.send_cmd(motor_lib.DIAG_BR_CMD)
                    time.sleep(0.5)
                    #Rot about BL until edge event
                    self.motor.send_cmd(motor_lib.ROT_BL_CMD)
                    state = 'CHECK_LEFT'
                elif right_edge:
                    #diag over BL burst
                    self.motor.send_cmd(motor_lib.DIAG_BL_CMD)
                    time.sleep(0.5)
                    #Rot about BR until edge event
                    self.motor.send_cmd(motor_lib.ROT_BR_CMD)
                    state = 'CHECK_RIGHT'
                else:
                    state = 'ERROR'

            elif state == 'CHECK_LEFT':
                if left_edge:
                    #stop cmd
                    self.motor.send_cmd(motor_lib.STOP_CMD)
                    state = 'BACKING'
                elif right_edge:
                    #stop cmd
                    self.motor.send_cmd(motor_lib.STOP_CMD)
                    state = 'ALIGNING'

            elif state == 'CHECK_RIGHT':
                if left_edge:
                    #stop cmd
                    self.motor.send_cmd(motor_lib.STOP_CMD)
                    state = 'ALIGNING'
                elif right_edge:
                    #stop cmd
                    self.motor.send_cmd(motor_lib.STOP_CMD)
                    state = 'BACKING'

            elif state == 'ALIGNING':
                if left_edge:
                    #Rot L
                    self.motor.send_cmd(motor_lib.ROT_L_CMD)
                elif right_edge:
                    #Rot R
                    self.motor.send_cmd(motor_lib.ROT_R_CMD)
                else:
                    #forward
                    self.motor.send_cmd(motor_lib.FORWARD_CMD)

            elif state == 'DONE':
                # print('Mission accomplished!')
                return 0

            else:
                print('!Bad State Logic')
                return -3

            if state == 'ERROR':
                self.motor.send_cmd(motor_lib.STOP_CMD)
                print('!Hit Error State')
                return -2

            loop_count += 1

        self.motor.send_cmd(motor_lib.STOP_CMD)
        print('!Timeout')
        return -1

    def follow(self):
        ''''''
        read_buff=bytearray(16)
        pixy_ts = time.time()
        EDGE_DETECTED = False
        TARGET_STATUS = 'NONE'
        while 1:
            #rgb logic
            rl, gl, bl, cl = self.rgb_left.color_raw
            rr, gr, br, cr = self.rgb_right.color_raw

            left_purple = 50>=cl>17 and 25>=rl>8
            right_purple = 50>=cr>17 and 25>=rr>8
            left_yellow = cl>50 and rl>25
            right_yellow = cr>50 and rr>25
            EDGE_DETECTED = left_yellow or right_yellow

            #pixy logic
            with self.pixy:
                self.pixy.write_then_readinto(bytearray(pixy_lib.get_blocks_cmd), read_buff)
            block_msg = pixy_lib.GetBlocksMsg(read_buff)
            if block_msg.type_code == 33 and block_msg.payload_length > 0:
                pixy_ts = time.time()
                [x_state, y_state, size_state] = pixy_lib.evaluate_cc_block(block_msg)
                if x_state == ord('G') and size_state == ord('G'):
                    TARGET_STATUS = 'LOCKED'
                elif x_state == ord('G'):
                    TARGET_STATUS = 'CENTERED'
                elif size_state == ord('G'):
                    TARGET_STATUS = 'RANGED'
                else:
                    TARGET_STATUS = 'SIGHTED'

            if time.time() > pixy_ts + 5.0:
                TARGET_STATUS = 'NONE'

            ##
            # Motor Logic
            if TARGET_STATUS == 'LOCKED':
                motor_cmd = motor_lib.STOP_CMD
                print("I: Motor Logic has achieved objective")
                sys.exit()
            elif TARGET_STATUS == 'CENTERED' and not EDGE_DETECTED:
                motor_cmd = motor_lib.FORWARD_CMD
            elif TARGET_STATUS == 'RANGED':
                if x_state == ord('R'):
                    motor_cmd = motor_lib.ROT_R_CMD
                elif x_state == ord('L'):
                    motor_cmd = motor_lib.ROT_L_CMD
                else:
                    # print('E: bad in-range logic')
                    pass
            elif TARGET_STATUS == 'SIGHTED':
                if x_state == ord('R'):
                    motor_cmd = motor_lib.ROT_R_CMD
                elif x_state == ord('L'):
                    motor_cmd = motor_lib.ROT_L_CMD
                else:
                    print('E: bad in-sight logic')
            else:
                motor_cmd = motor_lib.ROT_R_CMD

            self.motor.send_cmd(motor_cmd)

            # print(f'{TARGET_STATUS}')
            time.sleep(0.1)

if __name__ == '__main__':
    pass
