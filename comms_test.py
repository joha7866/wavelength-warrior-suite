#!/usr/bin/env python3
'''This program implements a simple data collection behavior for the Wavelenght warrior.

This is useful for I2C development and debug without the complications of motor activity.

The only external dependency in the pixy_smbus library for pixy commands.
The sensors currently implemented are:
 - 1x RGB sensor
 - The pixy
 '''
import time
import board
import busio
import adafruit_tca9548a
import adafruit_tcs34725
from adafruit_bus_device.i2c_device import I2CDevice

import modules.pixy_smbus as pixy_lib

if __name__ == "__main__":
    #use busio library to start our I2C bus
    with busio.I2C(board.SCL, board.SDA) as bus:

        #initialize our i2c devices with busses and addresses
        mux = adafruit_tca9548a.TCA9548A(bus)
        rgb1 = adafruit_tcs34725.TCS34725(mux[2])
        pixy = I2CDevice(mux[3], 0x54)

        #initialize variables for the loop
        read_buff = bytearray(16) #I2CDevice interactions require data in bytearray form
        pixy_ts = time.time()
        TARGET_STATUS = 'NONE'

        #wrapping main loop in try/except to enable clean ctrl+C exit
        try:
            #main loop
            while True:
                #get rgb sensor data
                lux1 = rgb1.lux
                print(f"RGB1 Lux: {lux1}")
                time.sleep(0.2)

                #get pixy data
                with pixy:
                    pixy.write_then_readinto(bytearray(pixy_lib.get_blocks_cmd), read_buff)
                    print('Did read pixy')
                block_msg = pixy_lib.GetBlocksMsg(read_buff)
                if block_msg.type_code == 33 and block_msg.payload_length > 0:
                    pixy_ts = time.time()
                    [x_state, y_state, size_state] = pixy_lib.evaluate_cc_block(block_msg)
                    if x_state == ord('G') and size_state == ord('G'):
                        TARGET_STATUS = 'LOCKED'
                    elif x_state == ord('G'):
                        TARGET_STATUS = 'CENTERED'
                    elif size_state == ord('G'):
                        TARGET_STATUS = 'RANGED'
                    else:
                        TARGET_STATUS = 'SIGHTED'

                if time.time() > pixy_ts + 5.0:
                    TARGET_STATUS = 'NONE'

                #print status each loop
                print(f'Target Status: {TARGET_STATUS}')
                time.sleep(0.2)
        except KeyboardInterrupt:
            print('graceful exit')
