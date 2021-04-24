#!/usr/bin/env python3
''''''
import time
import board
import busio
import adafruit_tca9548a
from adafruit_bus_device.i2c_device import I2CDevice

LASER_CTRL_ADDR = 0x53

class LaserController(object):
    def __init__(self, bus):
        self.laser = I2CDevice(bus, LASER_CTRL_ADDR, probe=False)
        self.active_cmd = 'x'

    def send_cmd(self, cmd):
        read_buff = bytearray(16)

        with self.laser:
            self.laser.write_then_readinto(cmd, read_buff)
        self.active_cmd = read_buff[0]

        return self.active_cmd
