#!/usr/bin/env python3
'''The embedded program for the Wavelength Warrior.
Initialize the robot, then wait until the automation switch comes on.

The brain is designed for two conditions not LOST and LOST.
not LOST: A choreographed path through the entire map, attempting to do the check() method at each valid location for
a balloon.
LOST: A random action selector, designed to move the robot in small steps and then rotate to check for valid targets.

This should be a very abstract flow: the majority of code should just be calls to the Robot class's implemented
actions.

NOTE: This script begins on reboot in the chrontab; be sure to kill it before beginning debug.
'''
import sys
import time
import board
import busio
import digitalio
from random import randint
import adafruit_tca9548a
import adafruit_tcs34725
import adafruit_hcsr04
from adafruit_bus_device.i2c_device import I2CDevice

import modules.motor_smbus as motor_lib
from modules.robot import Robot
import modules.laser as laser_lib

switch=digitalio.DigitalInOut(board.D11)
switch.direction = digitalio.Direction.INPUT

with busio.I2C(board.SCL, board.SDA) as bus:
    robot = Robot(bus)
    start_ts = time.time()
    LOST = False

    while True:
        while switch.value is False:
            pass
        time.sleep(0.3)

        while switch.value is True:
            try:
                if not LOST:
                    robot.leave_start() #leaves you at intersection
                    robot.do_motor_burst(motor_lib.FORWARD_CMD) #enter intersection
                    robot.do_align() #aligns to far side
                    robot.do_turn(angle=motor_lib.RIGHT_90_DIR)
                    robot.do_motor_burst(motor_lib.TRAN_R_CMD)
                    robot.check()
                    robot.fire(cmd=laser_lib.LASER_FIRE_CMD)

                    robot.do_align()
                    robot.do_motor_burst(motor_lib.BACKWARD_CMD)
                    robot.do_turn(angle=motor_lib.RIGHT_180_DIR)
                    robot.do_fwd_deflect_edge(stop_purple=0)
                    robot.do_motor_burst(motor_lib.BACKWARD_CMD)
                    # robot.do_align()
                    # robot.do_motor_burst(motor_lib.BACKWARD_CMD)
                    robot.do_turn(angle=motor_lib.RIGHT_90_DIR)
                    robot.do_fwd_deflect_edge(stop_purple=0)
                    robot.do_motor_burst(motor_lib.BACKWARD_CMD)
                    # robot.do_align()
                    # robot.do_motor_burst(motor_lib.BACKWARD_CMD)
                    robot.check()
                    robot.fire(cmd=laser_lib.LASER_FIRE_CMD)

                    robot.do_motor_burst(motor_lib.BACKWARD_CMD)
                    robot.do_turn(angle=motor_lib.RIGHT_90_DIR)
                    robot.check()
                    robot.fire(cmd=laser_lib.LASER_FIRE_CMD)

                    robot.do_motor_burst(motor_lib.BACKWARD_CMD)
                    robot.do_turn(angle=motor_lib.LEFT_180_DIR)
                    robot.do_align()
                    robot.do_motor_burst(motor_lib.BACKWARD_CMD)
                    robot.do_turn(angle=motor_lib.LEFT_90_DIR)
                    robot.do_fwd_deflect_edge(stop_purple=0)
                    robot.do_motor_burst(motor_lib.BACKWARD_CMD)
                    robot.do_turn(angle=motor_lib.LEFT_90_DIR)
                    robot.check()
                    robot.fire(cmd=laser_lib.LASER_FIRE_CMD)

                    robot.do_motor_burst(motor_lib.BACKWARD_CMD)
                    robot.do_turn(angle=motor_lib.LEFT_180_DIR)
                    robot.do_align()
                    robot.do_motor_burst(motor_lib.BACKWARD_CMD)
                    robot.do_turn(angle=motor_lib.RIGHT_90_DIR)
                    robot.do_fwd_deflect_edge(stop_purple=1)
                    robot.do_motor_burst(motor_lib.FORWARD_CMD)
                    robot.do_align()
                    robot.do_turn(angle=motor_lib.RIGHT_90_DIR)
                    robot.do_fwd_deflect_edge(stop_purple=1)
                    robot.do_motor_burst(motor_lib.FORWARD_CMD)
                    robot.do_fwd_deflect_edge(stop_purple=1)
                    robot.do_motor_burst(motor_lib.FORWARD_CMD)
                    robot.do_fwd_deflect_edge(stop_purple=1)
                    robot.do_motor_burst(motor_lib.BACKWARD_CMD)
                    robot.do_turn(angle=motor_lib.LEFT_90_DIR)
                    robot.do_fwd_deflect_edge(stop_purple=0)
                else:
                    choice = randint(0,3)
                    if choice == 0:
                        time.sleep(0.5)
                    elif choice == 1:
                        robot.do_align_check(timeout=1.0)
                        robot.do_motor_burst(motor_lib.BACKWARD_CMD)
                    elif choice == 2:
                        robot.do_turn(angle=motor_lib.RIGHT_90_DIR)
                    elif choice == 3:
                        robot.follow(timeout=5.0)
                        robot.fire(cmd=laser_lib.LASER_FIRE_CMD)

            except KeyboardInterrupt:
                print('!Interrupt')
                break
            except IOError:
                print('!IOerror')
                LOST = True

        robot.stop()
        time.sleep(0.3)
