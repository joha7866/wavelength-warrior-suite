#!/usr/bin/env python3
'''This module implements command definitions for the motor driver I2C communications.'''
import time
import board
import busio
import adafruit_tca9548a
from adafruit_bus_device.i2c_device import I2CDevice

MOTOR_CTRL_ADDR = 0x69

#motor info
FORWARD_CMD = bytearray([ord('F')])
BACKWARD_CMD = bytearray([ord('F')])
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
STOP_CMD = bytearray([ord('S')])
ERROR_CMD = bytearray([ord('E')])
POLL_CMD = bytearray([ord('.')])

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
