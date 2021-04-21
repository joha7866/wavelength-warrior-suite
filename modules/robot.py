#!/usr/bin/env python3
'''This module'''
import board
import busio
import adafruit_tca9548a
import adafruit_tcs34725
import adafruit_hcsr04
from adafruit_bus_device.i2c_device import I2CDevice

import motor_smbus as motor_lib

class Robot(object):
    def __init__(self, bus):
        self.bus = bus

        self.mux = adafruit_tca9548a.TCA9548A(bus)
        self.rgb_left = adafruit_tcs34725.TCS34725(self.mux[7])
        self.rgb_right = adafruit_tcs34725.TCS34725(self.mux[5])
        self.motor = motor_lib.MotorController(self.mux[0])

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
                print('Mission accomplished!')
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

if __name__ == '__main__':
    pass
