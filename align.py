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

if __name__ == "__main__":
    with busio.I2C(board.SCL, board.SDA) as bus:
        robot = Robot(bus)
        start_ts = time.time()

        while time.time() < start_ts + 30.0:

            try:
                robot.do_fwd_deflect_edge(pcount=2)
                # robot.do_align()
                # robot.do_turn(dir='left')
                # robot.do_align()
                # robot.do_turn(dir='right')
                # robot.cross_purples(count=2)
                # robot.follow()
                # robot.test_fire()
                # robot.do_turn(dir='left')
                # robot.do_align()
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

