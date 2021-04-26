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
from modules.robot import Robot
import modules.laser as laser_lib

if __name__ == "__main__":
    with busio.I2C(board.SCL, board.SDA) as bus:
        robot = Robot(bus)
        start_ts = time.time()

        while time.time() < start_ts + 30.0:

            try:
                robot.leave_start() #leaves you at intersection
                robot.do_motor_burst(motor_lib.FORWARD_CMD) #enter intersection
                robot.do_align() #aligns to far side
                robot.do_turn(angle=motor_lib.RIGHT_90_DIR)
                robot.follow()
                robot.fire(cmd=laser_lib.LASER_TEST_CMD)


                robot.stop()
                print('done')
                break
            except KeyboardInterrupt:
                print('!Interrupt')
                break
            except IOError:
                print('!IOerror')
                break

        robot.stop()

