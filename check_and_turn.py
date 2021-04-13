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
LEFT_US_PIN_PAIR = [board.D19,board.D26]
RIGHT_US_PIN_PAIR = [board.D16,board.D20]

if __name__ == "__main__":
    #setup
    left_us = adafruit_hcsr04.HCSR04(trigger_pin=LEFT_US_PIN_PAIR[0], echo_pin=LEFT_US_PIN_PAIR[1])
    right_us = adafruit_hcsr04.HCSR04(trigger_pin=RIGHT_US_PIN_PAIR[0], echo_pin=RIGHT_US_PIN_PAIR[1])

    read_buff = bytearray(16)
    loop_count = 0

    with busio.I2C(board.SCL, board.SDA) as bus:
        mux = adafruit_tca9548a.TCA9548A(bus)
        rgb_left = adafruit_tcs34725.TCS34725(mux[5])
        rgb_right = adafruit_tcs34725.TCS34725(mux[7])
        motor = I2CDevice(mux[0], MOTOR_CTRL_ADDR, probe=False)


        while 1:
            # #us info
            # try:
            #     left_dist = left_us.distance
            # except RuntimeError:
            #     left_dist = -1.0
            # try:
            #     right_dist = right_us.distance
            # except RuntimeError:
            #     right_dist = -1.0
            # print(f'Dist L:{left_dist:.1f}, R:{right_dist:.1f}')

            #burst back
            with motor:
                motor.write_then_readinto(motor_lib.BACKWARD_CMD, read_buff)
            time.sleep(0.5)
            with motor:
                motor.write_then_readinto(motor_lib.STOP_CMD, read_buff)

            #rotate 90
            with motor:
                motor.write_then_readinto(motor_lib.ROT_R_CMD, read_buff)
            time.sleep(1.33)
            with motor:
                motor.write_then_readinto(motor_lib.STOP_CMD, read_buff)
            sys.exit()
            time.sleep(2)
