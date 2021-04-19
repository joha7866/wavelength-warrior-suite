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
                r1, g1, b1 = rgb_left.color_rgb_bytes
                lux_l = rgb_left.lux
                r2, g2, b2 = rgb_right.color_rgb_bytes
                lux_r = rgb_right.lux
                left_edge = lux_l>700
                right_edge = lux_r>700
                if loop_count%4 == 0:
                    print(f'Lux L:{lux_l:.1f}, R:{lux_r:.1f}')
                    print(f'RGB L:({r1},{g1},{b1}), R:({r2},{g2},{b2})')

                time.sleep(0.24)
                loop_count += 1
    except KeyboardInterrupt:
        print('exited gracefully')
