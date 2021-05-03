#!/usr/bin/env python3
''''''
import time
import board
import busio
import adafruit_tca9548a
from adafruit_bus_device.i2c_device import I2CDevice

LASER_CTRL_ADDR = 0x53

LASER_TEST_CMD = bytearray([ord('T')])
LASER_FIRE_CMD = bytearray([ord('P')])
LASER_CANCEL_CMD = bytearray([ord('C')])

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

if __name__ == "__main__":
    read_buff = bytearray(16)
    with busio.I2C(board.SCL, board.SDA) as bus:
        mux = adafruit_tca9548a.TCA9548A(bus)
        laser = I2CDevice(mux[3], 0x53, probe=False)
        try:
            while 1:
                cmd = input('cmd>>')
                cmd = bytearray([ord(ch) for ch in cmd])
                print(f'Txing: "{cmd}"')
                with laser:
                    laser.write_then_readinto(bytearray(cmd), read_buff)
                print(f'Rxed:  "{read_buff}"')
        except KeyboardInterrupt:
            with laser:
                laser.write_then_readinto(bytearray([ord('C')]), read_buff)
            print('exited gracefully')
