#!/usr/bin/env python3
import time
# import RPi.GPIO as GP #unnecessary with circuitpython?
import board
import busio
import adafruit_tca9548a
import adafruit_tcs34725
from adafruit_bus_device.i2c_device import I2CDevice

import modules.ultrasonic_gpio as us_lib
import modules.pixy_smbus as pixy_lib

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
        rgb1 = adafruit_tcs34725.TCS34725(mux[2])

        motor = I2CDevice(mux[0], MOTOR_CTRL_ADDR)
        # laser = I2CDevice(mux[1], 0x18)
        pixy = I2CDevice(mux[7], PIXY_ADDR)

        # us_lib.setup_ultrasonic_system([LEFT_ULTRASONIC_PAIR, RIGHT_ULTRASONIC_PAIR])

        read_buff = bytearray(MAX_I2C_MSG_BYTES)

        pixy_ts = time.time()

        EDGE_DETECTED = False
        TARGET_STATUS = 'NONE'
        try:
            while True:
                #rgb logic
                [lux1] = check_rgbs([rgb1])
                print(f'L1: {lux1}')
                EDGE_DETECTED = (lux1 < 750)

                #pixy logic
                pixy.write_then_readinto(pixy_lib.get_blocks_cmd, read_buff)
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

                #motor logic
                # motor.write_then_readinto([ord('.')], read_buff)
                # curr_cmd = chr(read_buff[0])
                # available = read_buff[1] == ord('D')
                # if available:
                #     if EDGE_DETECTED:
                #         motor.write([ord('S')])
                #     else:
                #         motor.write([ord('F')])
                if TARGET_STATUS == 'LOCKED':
                    motor.write([ord('S')])
                    print("I: Motor Logic has achieved objective")
                elif TARGET_STATUS == 'CENTERED' and not EDGE_DETECTED:
                    motor.write([ord('F')])
                elif TARGET_STATUS == 'RANGED':
                    if x_state == ord('R'):
                        motor.write([ord('R')])
                    elif x_state == ord('L'):
                        motor.write([ord('L')])
                    else:
                        print('E: bad in-range logic')
                elif TARGET_STATUS == 'SIGHTED':
                    if x_state == ord('R'):
                        motor.write([ord('R')])
                    elif x_state == ord('L'):
                        motor.write([ord('F')])
                    else:
                        print('E: bad in-sight logic')
                else:
                    motor.write([ord('S')])
                    print('I: Nothing to see here')

                time.sleep(1)
        finally:
            motor.write([ord('s')])
            print('graceful exit')
