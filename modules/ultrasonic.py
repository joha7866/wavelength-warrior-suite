#!/usr/bin/env python3
'''Debug module for ultrasonic sensors.'''
import sys
import time
import board
import busio
import adafruit_tca9548a
import adafruit_tcs34725
import adafruit_hcsr04
from adafruit_bus_device.i2c_device import I2CDevice

#gpio definitions
LEFT_US_PIN_PAIR = [board.D19,board.D26]
RIGHT_US_PIN_PAIR = [board.D16,board.D20]

if __name__ == "__main__":
    #setup
    left_us = adafruit_hcsr04.HCSR04(trigger_pin=LEFT_US_PIN_PAIR[0], echo_pin=LEFT_US_PIN_PAIR[1])
    right_us = adafruit_hcsr04.HCSR04(trigger_pin=RIGHT_US_PIN_PAIR[0], echo_pin=RIGHT_US_PIN_PAIR[1])

    try:
        while 1:
            #us info
            try:
                time.sleep(1)
                left_dist = left_us.distance
            except RuntimeError:
                left_dist = -1.0
            try:
                time.sleep(1)
                right_dist = right_us.distance
            except RuntimeError:
                right_dist = -1.0
            print(f'Dist L:{left_dist:.1f}, R:{right_dist:.1f}')

            time.sleep(2)
    except KeyboardInterrupt:
        print('graceful exit')
