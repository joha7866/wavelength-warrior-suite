#!/usr/bin/env python3
'''test'''
import sys
import time
import board
import busio
import adafruit_tca9548a
import adafruit_tcs34725
import adafruit_hcsr04
from adafruit_bus_device.i2c_device import I2CDevice

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
