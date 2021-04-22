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
    read_buff = bytearray(16)
    hit_count = 0

    with busio.I2C(board.SCL, board.SDA) as bus:
        mux = adafruit_tca9548a.TCA9548A(bus)
        rgb_left = adafruit_tcs34725.TCS34725(mux[5])
        rgb_right = adafruit_tcs34725.TCS34725(mux[7])
        motor = I2CDevice(mux[0], MOTOR_CTRL_ADDR, probe=False)

        lux_l = rgb_left.lux
        lux_r = rgb_right.lux
        time.sleep(0.5)

        with motor:
            motor.write_then_readinto(motor_lib.FORWARD_CMD, read_buff)
        active_cmd = chr(read_buff[0])

        while 1:
            #read rgbs
            rl, gl, bl, cl = rgb_left.color_raw
            rr, gr, br, cr = rgb_right.color_raw

            left_purple = 50>=cl>20 and rl>20
            right_purple = 50>=cr>20 and rr>20
            left_yellow = cr>50
            right_yellow = cr>50

            if left_yellow or right_yellow:
                with motor:
                    motor.write_then_readinto(motor_lib.STOP_CMD, read_buff)
                active_cmd = chr(read_buff[0])
                print('stopped on edge')
                sys.exit()

            if left_purple or right_purple:
                hit_count += 1
                if hit_count >= 2:
                    with motor:
                        motor.write_then_readinto(motor_lib.STOP_CMD, read_buff)
                    active_cmd = chr(read_buff[0])
                    print('thru')
                    sys.exit()
                time.sleep(0.4)
