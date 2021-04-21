#!/usr/bin/env python3
import time
import sys
# import RPi.GPIO as GP #unnecessary with circuitpython?
import board
import busio
import adafruit_tca9548a
import adafruit_tcs34725
from adafruit_bus_device.i2c_device import I2CDevice

import modules.pixy_smbus as pixy_lib
import modules.motor_smbus as motor_lib

#i2c addressses
MOTOR_CTRL_ADDR = 0x69
PIXY_ADDR = 0x54
MAX_I2C_MSG_BYTES = 16

#gpio definitions
LEFT_ULTRASONIC_PAIR = [19,26]
RIGHT_ULTRASONIC_PAIR = [16,20]

def check_rgbs(dev_list):
    results = []
    for dev in dev_list:
        results.append(dev.lux)
    return results

if __name__ == "__main__":
    with busio.I2C(board.SCL, board.SDA) as bus:

        mux = adafruit_tca9548a.TCA9548A(bus)
        rgb_left = adafruit_tcs34725.TCS34725(mux[5])
        rgb_right = adafruit_tcs34725.TCS34725(mux[7])

        motor = I2CDevice(mux[0], MOTOR_CTRL_ADDR, probe=False)
        laser = I2CDevice(mux[3], 0x53, probe=False)
        pixy = I2CDevice(mux[4], PIXY_ADDR)

        # us_lib.setup_ultrasonic_system([LEFT_ULTRASONIC_PAIR, RIGHT_ULTRASONIC_PAIR])

        read_buff = bytearray(16)

        pixy_ts = time.time()

        EDGE_DETECTED = False
        TARGET_STATUS = 'NONE'
        try:
            while True:
                #rgb logic
                lux_l = rgb_left.lux
                lux_r = rgb_right.lux
                left_edge = (lux_l > 800)
                right_edge = (lux_r > 800)
                EDGE_DETECTED = left_edge or right_edge

                #pixy logic
                with pixy:
                    pixy.write_then_readinto(bytearray(pixy_lib.get_blocks_cmd), read_buff)
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

                ##
                # Motor Logic
                if TARGET_STATUS == 'LOCKED':
                    motor_cmd = motor_lib.STOP_CMD
                    print("I: Motor Logic has achieved objective")
                    sys.exit()
                elif TARGET_STATUS == 'CENTERED' and not EDGE_DETECTED:
                    motor_cmd = motor_lib.FORWARD_CMD
                elif TARGET_STATUS == 'RANGED':
                    if x_state == ord('R'):
                        motor_cmd = motor_lib.ROT_R_CMD
                    elif x_state == ord('L'):
                        motor_cmd = motor_lib.ROT_L_CMD
                    else:
                        print('E: bad in-range logic')
                elif TARGET_STATUS == 'SIGHTED':
                    if x_state == ord('R'):
                        motor_cmd = motor_lib.ROT_R_CMD
                    elif x_state == ord('L'):
                        motor_cmd = motor_lib.ROT_L_CMD
                    else:
                        print('E: bad in-sight logic')
                else:
                    motor_cmd = motor_lib.ROT_R_CMD

                with motor:
                    motor.write_then_readinto(motor_cmd, read_buff)

                print(f'{TARGET_STATUS}')
                time.sleep(0.3)

        finally:
            with motor:
                motor.write(motor_lib.STOP_CMD)
            print('graceful exit')
