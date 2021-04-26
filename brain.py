#!/usr/bin/env python3
''''''
import time
import board
import busio
import digitalio

from modules.robot import Robot

switch=digitalio.DigitalInOut(board.D11)
switch.direction = digitalio.Direction.INPUT

with busio.I2C(board.SCL, board.SDA) as bus:
    robot = Robot(bus)

    while True:
        while switch.value is True:
            robot.leave_start() #leaves you at intersection
            if switch.value is False:
                break
            robot.do_motor_burst(motor_lib.FORWARD_CMD) #enter intersection
            if switch.value is False:
                break
            robot.do_align() #aligns to far side
            if switch.value is False:
                break
            robot.do_turn(angle=motor_lib.RIGHT_90_DIR)
            if switch.value is False:
                break
            robot.follow()
            if switch.value is False:
                break
            robot.fire(cmd=laser_lib.LASER_TEST_CMD)
            sys.exit()
        print('STOP')
        robot.stop()
        while switch.value is False:
            pass
        time.sleep(0.5)
