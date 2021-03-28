#!/usr/bin/env python3
'''Manage Pixy, RGB Sense Arduino, and Ultrasonics, make decisions, then command motors'''
import time
from smbus2 import SMBus, i2c_msg

import modules.pixy_smbus as pixy_lib
import modules.ultrasonic_gpio as us_lib
import modules.motor_smbus as motor_lib

#i2c addressses
MOTOR_CTRL_ADDR = 0x69
RGB_SENSE_ADDR = 0x55
PIXY_ADDR = 0x54
MAX_I2C_MSG_BYTES = 16

#ultrasonic gpios
LEFT_ULTRASONIC_PAIR = [5,6]
RIGHT_ULTRASONIC_PAIR = [23,24]

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
    ultrasonic_timestamp = time.time()
    turn_timestamp = time.time()
    with SMBus(1) as bus:
        try:
            #loop
            while GOOD_TO_GO:
                #check rgb sensor every loop
                resp = send_i2c_cmd(bus, RGB_SENSE_ADDR, POLL_CMD)
                if list(resp)[0] != 0:
                    send_i2c_cmd(bus, MOTOR_CTRL_ADDR, STOP_CMD)
                    print('Lane edge detected!')
                    LANE_EDGE_FLAG = True

                #check pixy every loop
                resp = send_i2c_cmd(bus, PIXY_ADDR, get_blocks_cmd)
                block_msg = user_pixy.GetBlocksMsg(resp)
                if block_msg.type_code == 33 and block_msg.payload_length != 0:
                    [x_state, y_state, size_state] = user_pixy.evaluate_cc_block(block_msg)
                    TARGET_IN_SIGHT = True
                else:
                    TARGET_IN_SIGHT = False

                #check ultrasonics only when necessary
                if (LANE_EDGE_FLAG or time.time() > ultrasonic_timestamp+1) and not TURN_ACTIVE_FLAG:
                    time.sleep(0.05)
                    dist_left = measure_ultrasonic_distance(LEFT_ULTRASONIC_PAIR)
                    time.sleep(0.05)
                    dist_right = measure_ultrasonic_distance(RIGHT_ULTRASONIC_PAIR)
                    ultrasonic_timestamp = time.time()
                    print(f"Distance left: {dist_left:.1f} cm")
                    print(f"Distance right: {dist_right:.1f} cm")

                #update motor command per-loop
                if TURN_ACTIVE_FLAG and time.time() > turn_timestamp+2:
                    send_i2c_cmd(bus, MOTOR_CTRL_ADDR, STOP_CMD)
                    print('Turn complete')
                else:
                    resp = send_i2c_cmd(bus, MOTOR_CTRL_ADDR, POLL_CMD)
                    available = list(resp)[1] == ord('D')
                    if LANE_EDGE_FLAG:
                        if dist_left < SIDE_DIST_TOL_CM:
                            send_i2c_cmd(bus, MOTOR_CTRL_ADDR, RIGHT_90_CMD)
                            turn_timestamp = time.time()
                            TURN_ACTIVE_FLAG = True
                        elif dist_right < SIDE_DIST_TOL_CM:
                            send_i2c_cmd(bus, MOTOR_CTRL_ADDR, RIGHT_90_CMD)
                            turn_timestamp = time.time()
                            TURN_ACTIVE_FLAG = True
                        else:
                            print('E: RGB active, but no clear turn condition!')
                            GOOD_TO_GO = False
                    else:
                        if list(resp)[0] != ord('F'):
                            send_i2c_cmd(bus, MOTOR_CTRL_ADDR, FORWARD_CMD)


        except KeyboardInterrupt:
            print('T: Terminated by user')
            send_i2c_cmd(bus, MOTOR_CTRL_ADDR, STOP_CMD)
            print('Stop command sent')
        
        except:
            print('T: Terminated with unexpected error')


    if not GOOD_TO_GO:
        print('I: Logical termination')

    us_lib.cleanup()
    print('Graceful exit complete')
