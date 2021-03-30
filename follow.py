#!/usr/bin/env python3
'''Manage Pixy, RGB Sense Arduino, and Ultrasonics, make decisions, then command motors'''
import time
from smbus2 import SMBus, i2c_msg

from modules.i2c import *
import modules.pixy_smbus as pixy_lib
import modules.ultrasonic_gpio as us_lib
import modules.motor_smbus as motor_lib

#ultrasonic gpios
LEFT_ULTRASONIC_PAIR = [19,26]
RIGHT_ULTRASONIC_PAIR = [16,20]

#behavior metrics
SIDE_PROX_THRESH_CM = 40


if __name__ == '__main__':
    #setup
    us_lib.setup_ultrasonic_system([LEFT_ULTRASONIC_PAIR,RIGHT_ULTRASONIC_PAIR])

    GOOD_TO_GO = True
    LANE_EDGE_FLAG = False
    TARGET_IN_SIGHT = False
    TARGET_ALIGNED = False
    TURN_ACTIVE_FLAG = False
    RGB_DATA_GOOD_FLAG = True
    TARGET_IN_SIGHT = False
    TARGET_CENTERED = False
    TARGET_IN_RANGE = False
    TARGET_LOCKED = False
    TARGET_STATUS = 'NONE'
    ultrasonic_timestamp = time.time()
    turn_timestamp = time.time()
    edge_timestamp = time.time()
    pixy_timestamp = time.time()
    loop_count = 0
    x_state =''
    y_state =''
    size_state =''
    with SMBus(1) as bus:
        try:
            #loop
            while GOOD_TO_GO:
                print(f'>>>NEW LOOP: {loop_count}')

                #check pixy every loop
                resp = send_i2c_cmd(bus, PIXY_ADDR, pixy_lib.get_blocks_cmd)
                if not list(resp):
                    print('E: Pixy data failed. Keeping latest state.')
                else:
                    block_msg = pixy_lib.GetBlocksMsg(resp)
                    if block_msg.type_code == 33 and block_msg.payload_length != 0:
                        pixy_timestamp = time.time()
                        [x_state, y_state, size_state] = pixy_lib.evaluate_cc_block(block_msg)
                        TARGET_STATUS = 'SIGHTED'
                        print('I: Target in sight')
                        print(f'I: [x, size] [{chr(x_state)}, {chr(size_state)}]')

                        if x_state == ord('G') and size_state == ord('G'):
                            TARGET_STATUS = 'LOCKED'
                            print('I: TARGET IN RANGE!!!')
                        elif size_state == ord('G'):
                            TARGET_STATUS = 'RANGED'
                        elif x_state == ord('G'):
                            TARGET_STATUS = 'CENTERED'

                if time.time() > pixy_timestamp+5.0:
                    TARGET_STATUS = 'NONE'

                #motor logic
                if TARGET_STATUS == 'LOCKED':
                    send_i2c_cmd(bus, MOTOR_CTRL_ADDR, motor_lib.STOP_CMD)
                    print("I: Motor Logic has achieved objective")
                elif TARGET_STATUS == 'CENTERED':
                    send_i2c_cmd(bus, MOTOR_CTRL_ADDR, motor_lib.FORWARD_CMD)
                elif TARGET_STATUS == 'RANGED':
                    if x_state == ord('R'):
                        send_i2c_cmd(bus, MOTOR_CTRL_ADDR, motor_lib.RIGHT_90_CMD)
                    elif x_state == ord('L'):
                        send_i2c_cmd(bus, MOTOR_CTRL_ADDR, motor_lib.LEFT_90_CMD)
                    else:
                        print('E: bad in-range logic')
                elif TARGET_STATUS == 'SIGHTED':
                    if x_state == ord('R'):
                        send_i2c_cmd(bus, MOTOR_CTRL_ADDR, motor_lib.RIGHT_90_CMD)
                    elif x_state == ord('L'):
                        send_i2c_cmd(bus, MOTOR_CTRL_ADDR, motor_lib.LEFT_90_CMD)
                    else:
                        print('E: bad in-sight logic')
                else:
                    send_i2c_cmd(bus, MOTOR_CTRL_ADDR, motor_lib.STOP_CMD)
                    print('I: Nothing to see here')

                print(f'I: target status {TARGET_STATUS}')
                time.sleep(0.1)
                loop_count += 1
        except KeyboardInterrupt:
            print('T: Terminated by user')
            send_i2c_cmd(bus, MOTOR_CTRL_ADDR, motor_lib.STOP_CMD)
            print('Stop command sent')

        # except:
        #     print('T: Terminated with unexpected error')


    if not GOOD_TO_GO:
        print('I: Logical termination')

    us_lib.cleanup()
    print('Graceful exit complete')
