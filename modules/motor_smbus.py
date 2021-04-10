#!/usr/bin/env python3
'''This module implements command definitions for the motor driver I2C communications.'''

#motor info
FORWARD_CMD =   bytearray([ord('F')])
LEFT_CMD =      bytearray([ord('L')])
RIGHT_CMD =     bytearray([ord('R')])
STOP_CMD =      bytearray([ord('S')])
ERROR_CMD =     bytearray([ord('E')])
POLL_CMD =      bytearray([ord('.')])
