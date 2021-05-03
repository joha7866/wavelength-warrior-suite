#!/usr/bin/env python3
'''This module implements command definitions for the motor driver I2C communications.'''
import time
import math
import board
import busio
import adafruit_tca9548a
from adafruit_bus_device.i2c_device import I2CDevice

MOTOR_CTRL_ADDR = 0x69

#motor info
FORWARD_CMD = bytearray([ord('F')])
BACKWARD_CMD = bytearray([ord('B')])
ROT_L_CMD = bytearray([ord(ch) for ch in 'RL'])
ROT_R_CMD = bytearray([ord(ch) for ch in 'RR'])
ROT_FL_CMD = bytearray([ord(ch) for ch in 'RFL'])
ROT_FR_CMD = bytearray([ord(ch) for ch in 'RFR'])
ROT_BL_CMD = bytearray([ord(ch) for ch in 'RBL'])
ROT_BR_CMD = bytearray([ord(ch) for ch in 'RBR'])
DIAG_FL_CMD = bytearray([ord(ch) for ch in 'DFL'])
DIAG_FR_CMD = bytearray([ord(ch) for ch in 'DFR'])
DIAG_BL_CMD = bytearray([ord(ch) for ch in 'DBL'])
DIAG_BR_CMD = bytearray([ord(ch) for ch in 'DBR'])
TRAN_L_CMD = bytearray([ord(ch) for ch in 'TL'])
TRAN_R_CMD = bytearray([ord(ch) for ch in 'TR'])
STOP_CMD = bytearray([ord('S')])
ERROR_CMD = bytearray([ord('E')])
POLL_CMD = bytearray([ord('.')])

LEFT_90_DIR = math.pi/2
RIGHT_90_DIR = -math.pi/2
LEFT_180_DIR = math.pi
RIGHT_180_DIR = -math.pi

# ROT_90_DELAY = 1.33
ROT_90_DELAY = 1.45

class MotorController(object):
    def __init__(self, bus):
        self.motor = I2CDevice(bus, MOTOR_CTRL_ADDR, probe=False)
        self.active_cmd = 'x'

    def send_cmd(self, cmd):
        read_buff = bytearray(16)

        with self.motor:
            self.motor.write_then_readinto(cmd, read_buff)
        self.active_cmd = read_buff[0]

        if self.active_cmd == cmd[0]:
            return 0
        else:
            return self.active_cmd


if __name__ == "__main__":
    read_buff = bytearray(16)
    with busio.I2C(board.SCL, board.SDA) as bus:
        mux = adafruit_tca9548a.TCA9548A(bus)
        motor = I2CDevice(mux[0], MOTOR_CTRL_ADDR, probe=False)
        try:
            while 1:
                cmd = input('cmd>>')
                cmd = bytearray([ord(ch) for ch in cmd])
                print(f'Txing: "{cmd}"')
                with motor:
                    motor.write_then_readinto(bytearray(cmd), read_buff)
                print(f'Rxed:  "{read_buff}"')
        except KeyboardInterrupt:
            with motor:
                motor.write_then_readinto(STOP_CMD, read_buff)
            print('exited gracefully')
