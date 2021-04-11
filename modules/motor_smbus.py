#!/usr/bin/env python3
'''This module implements command definitions for the motor driver I2C communications.'''
import time
import board
import busio
import adafruit_tca9548a
from adafruit_bus_device.i2c_device import I2CDevice

MOTOR_CTRL_ADDR = 0x69

#motor info
FORWARD_CMD =   bytearray([ord('F')])
LEFT_CMD =      bytearray([ord('L')])
RIGHT_CMD =     bytearray([ord('R')])
STOP_CMD =      bytearray([ord('S')])
ERROR_CMD =     bytearray([ord('E')])
POLL_CMD =      bytearray([ord('.')])

if __name__ == "__main__":
    read_buff = bytearray(16)
    with busio.I2C(board.SCL, board.SDA) as bus:
        mux = adafruit_tca9548a.TCA9548A(bus)
        motor = I2CDevice(mux[0], MOTOR_CTRL_ADDR)
        while 1:
            cmd = input('cmd>>')
            cmd = bytearray([ord(ch) for ch in cmd])
            print(f'Txing: "{cmd}"')
            with motor:
                motor.write_then_readinto(bytearray(cmd), read_buff)
            print(f'Rxed:  "{read_buff}"')
