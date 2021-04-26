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
import modules.laser as laser_lib
import modules.rgb_sensor as rgb_lib

class Robot(object):
    '''This class implements the roboot in terms of sensors and actions.'''
    def __init__(self, bus):
        self.bus = bus

        self.mux = adafruit_tca9548a.TCA9548A(bus)
        self.imu = lsm6dsox.LSM6DSOX(self.mux[2])
        self.rgb_left = rgb_lib.RgbSensor(self.mux[7])
        self.rgb_center = rgb_lib.RgbSensor(self.mux[6])
        self.rgb_right = rgb_lib.RgbSensor(self.mux[5])
        self.motor = motor_lib.MotorController(self.mux[0])
        self.laser = laser_lib.LaserController(self.mux[3])
        self.pixy = pixy_lib.Pixy(self.mux[4], team='red')


    def do_motor_burst(self, cmd):
        self.motor.send_cmd(cmd)
        time.sleep(0.5)
        self.motor.send_cmd(motor_lib.STOP_CMD)


    def do_turn(self, angle=math.pi/2, timeout=10):
        '''test'''
        imu_lib.eval_angle(self.imu, self.motor, angle)


    def stop(self):
        ''''''
        if ord(self.motor.active_cmd) != motor_lib.STOP_CMD[0]:
            self.motor.send_cmd(motor_lib.STOP_CMD)


    def cross_purples(self, count=2):
        self.motor.send_cmd(motor_lib.FORWARD_CMD)
        hit_count = 0
        while 1:
            if self.rgb_left.yellow or self.rgb_right.yellow:
                self.motor.send_cmd(motor_lib.STOP_CMD)
                print('stopped on edge')
                return -1

            if self.rgb_left.purple or self.rgb_right.yellow:
                hit_count += 1
                if hit_count >= count:
                    self.motor.send_cmd(motor_lib.STOP_CMD)
                    return 0
                time.sleep(0.4)


    def do_forward(self):
        self.motor.send_cmd(motor_lib.FORWARD_CMD)


    def do_forward_with_deflect(self, timeout=10.0):
        self.motor.send_cmd(motor_lib.FORWARD_CMD)
        start_ts = time.time()

        while time.time() < start_ts + timeout:

            if self.rgb_left.yellow:
                self.motor.send_cmd(motor_lib.ROT_R_CMD)
                while self.rgb_left.yellow:
                    pass
                self.motor.send_cmd(motor_lib.FORWARD_CMD)

            elif self.rgb_right.yellow:
                self.motor.send_cmd(motor_lib.ROT_L_CMD)
                while self.rgb_left.yellow:
                    pass
                self.motor.send_cmd(motor_lib.FORWARD_CMD)

            else:
                pass

        self.motor.send_cmd(motor_lib.STOP_CMD)
        print('!TIMEOUT')
        return -1


    def do_fwd_deflect_edge(self, timeout=10.0, stop_purple=1):
        ''''''
        counter = 0
        self.motor.send_cmd(motor_lib.FORWARD_CMD)
        start_ts = time.time()

        while time.time() < start_ts + timeout:
            while self.rgb_center.black and time.time() < start_ts + timeout:

                if self.rgb_left.yellow:
                    self.motor.send_cmd(motor_lib.ROT_R_CMD)
                    while self.rgb_left.yellow:
                        pass
                    self.motor.send_cmd(motor_lib.FORWARD_CMD)

                elif self.rgb_right.yellow:
                    self.motor.send_cmd(motor_lib.ROT_L_CMD)
                    while self.rgb_left.yellow:
                        pass
                    self.motor.send_cmd(motor_lib.FORWARD_CMD)

                else:
                    pass

            if stop_purple > 0:
                if self.rgb_center.purple:
                    counter += 1
                    if counter >= stop_purple:
                        self.motor.send_cmd(motor_lib.STOP_CMD)
                        return 0

            if self.rgb_center.yellow:
                self.motor.send_cmd(motor_lib.STOP_CMD)
                return purple_count-counter

        self.motor.send_cmd(motor_lib.STOP_CMD)
        print('!TIMEOUT')
        return -1


    def leave_start(self, timeout=10):
        '''This is a brute-force action to exit the starting area.'''
        start_ts = time.time()

        while time.time() < start_ts + timeout:
            #go forward
            self.do_forward()
            #continue forward until any rgb_event
            while self.rgb_left.black and self.rgb_center.black and self.rgb_right.black:
                pass
            #stop
            self.stop()

            #finish if purple (we're on the main strech now)
            if self.rgb_center.purple:
                return 0
            #else align
            else:
                self.do_align

            #check again after the align
            if self.rgb_center.purple:
                return 0

            #burst backward and turn 90 degrees right to try the next wall
            self.do_motor_burst(motor_lib.BACKWARD_CMD)
            self.do_turn(motor_lib.RIGHT_90_DIR)


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
            if self.rgb_left.black and self.rgb_right.black:
                pass
            else:
                if self.rgb_left.yellow or self.rgb_left.yellow:
                    align_color = 'yellow'
                else:
                    align_color = 'purple'
                self.motor.send_cmd(motor_lib.STOP_CMD)
                state = 'BACKING'
                break

        while time.time() < start_ts+timeout:
            if align_color == 'purple':
                left_edge = self.rgb_left.purple
                right_edge = self.rgb_right.purple
            else:
                left_edge = self.rgb_left.yellow
                right_edge = self.rgb_right.yellow

            if left_edge and right_edge:
                self.motor.send_cmd(motor_lib.STOP_CMD)
                state = 'DONE'

            if state == 'BACKING':
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
                self.motor.send_cmd(motor_lib.STOP_CMD)
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
            #pixy logic
            with self.pixy:
                self.pixy.write_then_readinto(self.pixy.get_blocks_cmd, read_buff)
            block_msg = pixy_lib.GetBlocksMsg(read_buff)
            if block_msg.type_code == 33 and block_msg.payload_length > 0:
                pixy_ts = time.time()
                [x_state, _, size_state] = self.pixy.evaluate_cc_block(block_msg)
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
                return 0
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
                    # print('E: bad in-sight logic')
                    pass

            else:
                motor_cmd = motor_lib.ROT_R_CMD

            self.motor.send_cmd(motor_cmd)

            # print(f'{TARGET_STATUS}')
            time.sleep(0.01)


    def fire(self, cmd=laser_lib.LASER_TEST_CMD):
        self.motor.send_cmd(motor_lib.STOP_CMD)
        self.laser.send_cmd(cmd)
        start_ts = time.time()
        #while pixy sees balloon loop
        #Then send c
        object_present = True
        while object_present and time.time() < start_ts + 4.5:
            with self.pixy:
                self.pixy.write_then_readinto(self.pixy.get_blocks_cmd, read_buff)
            block_msg = pixy_lib.GetBlocksMsg(read_buff)
            if block_msg.type_code == 33 and block_msg.payload_length > 0:
                [x_state, y_state, size_state] = self.pixy.evaluate_cc_block(block_msg)
                if x_state != ord('G') or y_state != ord('G') or size_state != ord('G'):
                    object_present = False
                else:
                    object_present = True
            else:
                object_present = False
        self.laser.send_cmd(bytearray([ord('C')]))
        
        if time.time() >= start_ts + 4.5:
            return -1
        else:
            return 0


if __name__ == '__main__':
    pass
