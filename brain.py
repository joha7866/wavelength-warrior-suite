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
            robot.do_forward_with_deflect(timeout=2.0)
        print('STOP')
        robot.stop()
        while switch.value is False:
            pass
        time.sleep(0.5)
