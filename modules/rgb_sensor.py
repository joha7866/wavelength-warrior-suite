#!/usr/bin/env python3
'''This program attempts to legally navigate the course and lock a target.

It is a fundamental behavior program that:
 - Implements the RGB, Pixy, and US sensors
 - Actuates the motors
 - Attempts to navigate the course legally (but not necessarily logically)
'''
import sys
import time
import board
import busio
import adafruit_tca9548a
import adafruit_tcs34725
from adafruit_bus_device.i2c_device import I2CDevice


def RgbSensor(object):
    def __init__(self, bus):
        self.sensor = adafruit_tcs34725.TCS34725(bus)

    @property
    def black(self):
        r, g, b, c = self.sensor.color_raw
        return (c<17 or r<8)

    @property
    def purple(self):
        r, g, b, c = self.sensor.color_raw
        return (50>=c>17 and 25>=r>8)

    @property
    def yellow(self):
        r, g, b, c = self.sensor.color_raw
        return (c>50 and r>25)


if __name__ == "__main__":
    try:
        read_buff = bytearray(16)
        loop_count = 0

        with busio.I2C(board.SCL, board.SDA) as bus:
            mux = adafruit_tca9548a.TCA9548A(bus)
            rgb_left = adafruit_tcs34725.TCS34725(mux[7])
            rgb_right = adafruit_tcs34725.TCS34725(mux[5])

            while 1:
                #read rgbs
                rl, gl, bl, cl = rgb_left.color_raw
                rr, gr, br, cr = rgb_right.color_raw
                if loop_count%4 == 0:
                    print(f'Left:  R:{rl} G:{gl} B:{bl} C:{cl}')
                    print(f'Right: R:{rr} G:{gr} B:{br} C:{cr}')
                time.sleep(0.24)
                loop_count += 1
    except KeyboardInterrupt:
        print('exited gracefully')
