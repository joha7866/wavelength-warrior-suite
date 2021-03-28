#!/usr/bin/env python3
'''This module implements command definitions for the motor driver I2C communications.'''

#motor info
FORWARD_CMD = [ord('F')]
LEFT_90_CMD = [ord('L')]
RIGHT_90_CMD = [ord('R')]
STOP_CMD = [ord('S')]
CLEAR_ERROR_CMD = [ord('E')]
POLL_CMD = [ord('.')]
