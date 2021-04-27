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
from random import randint
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
        LOST = False

        while time.time() < start_ts + 30.0:

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
                    choice = randint(0,5);
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

