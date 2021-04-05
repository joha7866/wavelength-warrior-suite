#!/usr/bin/env python3
'''testing'''
import time
import board
import busio
import adafruit_tcs34725

def setup():
    i2c = busio.I2C(board.SCL, board.SDA)
    sensor = adafruit_tcs34725.TCS34725(i2c)
    return sensor

if __name__ == '__main__':
    sensor = setup()
    while True:
        lux = sensor.lux
        print(f'Lux: {lux}')
        time.sleep(1)
