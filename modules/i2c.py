#!/usr/bin/env python3
'''This module implements high-level I2C interactions.'''
import time
from smbus2 import SMBus, i2c_msg

__DEBUG_FLAG = False

#i2c addressses
MOTOR_CTRL_ADDR = 0x69
RGB_SENSE_ADDR = 0x55
PIXY_ADDR = 0x54
MAX_I2C_MSG_BYTES = 16


def send_i2c_cmd(bus, addr, cmd):
    '''Generic function to send a pair of receive, request commands to secondaries.
        Returns request data as an i2c_msg.'''
    try:
        write = i2c_msg.write(addr, cmd)
        read = i2c_msg.read(addr, MAX_I2C_MSG_BYTES)
        bus.i2c_rdwr(write, read)
        if __DEBUG_FLAG:
            print('Wrote: ',[chr(ch) for ch in cmd])
            print('Read:  ',[chr(ch) for ch in read])
    except IOError:
        report_ioerror(addr, cmd)
        read = []
    return read


def report_ioerror(addr, cmd):
    if addr == MOTOR_CTRL_ADDR:
        dest = 'Motor Control'
    elif addr == RGB_SENSE_ADDR:
        dest = 'RGB Sense'
    elif addr == PIXY_ADDR:
        dest = 'Pixy'
    else:
        dest = 'UNRECOGNIZED DEST'
    
    print(f'E: IOError with Dest: {dest}, Cmd: {str(cmd)}')


if __name__ == '__main__':
    addr = input('Dest addr (hex): ')
    print(int(addr))
