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
    FINAL_TARGET_SWEEP_FLAG = True
    ultrasonic_timestamp = time.time()
    turn_timestamp = time.time()
    edge_timestamp = time.time()
    loop_count = 0
    with SMBus(1) as bus:
        try:
            #loop
            while GOOD_TO_GO:
                print(f'>>>NEW LOOP: {loop_count}')

                #check rgb sensor every loop
                resp = send_i2c_cmd(bus, RGB_SENSE_ADDR, [ord('x')])
                if not list(resp):
                    if not TURN_ACTIVE_FLAG:
                        send_i2c_cmd(bus, MOTOR_CTRL_ADDR, motor_lib.STOP_CMD)
                    print('E: Bad RGB response')
                    RGB_DATA_GOOD_FLAG = False
                else:
                    RGB_DATA_GOOD_FLAG = True
                    print(f'I: RGB state {hex(list(resp)[0])}')
                    if list(resp)[0] != 0:
                        if not TURN_ACTIVE_FLAG:
                            send_i2c_cmd(bus, MOTOR_CTRL_ADDR, motor_lib.STOP_CMD)
                        print('Lane edge detected!')
                        LANE_EDGE_FLAG = True
                    else:
                        LANE_EDGE_FLAG = False


                #check pixy every loop
                resp = send_i2c_cmd(bus, PIXY_ADDR, pixy_lib.get_blocks_cmd)
                block_msg = pixy_lib.GetBlocksMsg(resp)
                if block_msg.type_code == 33 and block_msg.payload_length != 0:
                    [x_state, y_state, size_state] = pixy_lib.evaluate_cc_block(block_msg)
                    TARGET_IN_SIGHT = True
                    print('I: Target in sight')
                    print(f'I: [x, size] [{chr(x_state)}, {chr(size_state)}]')

                    if x_state == ord('G') and size_state == ord('G'):
                        TARGET_IN_RANGE = True
                        print('I: TARGET IN RANGE!!!')
                        send_i2c_cmd(bus, MOTOR_CTRL_ADDR, motor_lib.STOP_CMD)
                        time.sleep(10)
                    elif size_state == ord('G'):
                        FINAL_TARGET_SWEEP_FLAG = True
                else:
                    TARGET_IN_SIGHT = False


                #check ultrasonics only when necessary
                if (LANE_EDGE_FLAG or time.time() > ultrasonic_timestamp+1) and not TURN_ACTIVE_FLAG:
                    time.sleep(0.1)
                    dist_left = us_lib.measure_ultrasonic_distance(LEFT_ULTRASONIC_PAIR)
                    time.sleep(0.1)
                    dist_right = us_lib.measure_ultrasonic_distance(RIGHT_ULTRASONIC_PAIR)
                    ultrasonic_timestamp = time.time()
                    print(f"Distance left: {dist_left:.1f} cm")
                    print(f"Distance right: {dist_right:.1f} cm")

                #update motor command per-loop
                if FINAL_TARGET_SWEEP_FLAG:
                    if x_state =='R':
                        send_i2c_cmd(bus, MOTOR_CTRL_ADDR, motor_lib.RIGHT_90_CMD)
                    elif x_state =='L':
                        send_i2c_cmd(bus, MOTOR_CTRL_ADDR, motor_lib.LEFT_90_CMD)
                    else:
                        send_i2c_cmd(bus, MOTOR_CTRL_ADDR, motor_lib.STOP_CMD)
                        print("I: Motor Logic has achieved objective")
                elif TURN_ACTIVE_FLAG:
                    if time.time() > turn_timestamp+2.0:
                        send_i2c_cmd(bus, MOTOR_CTRL_ADDR, motor_lib.STOP_CMD)
                        TURN_ACTIVE_FLAG = False
                        print('Turn complete')
                else:
                    resp = send_i2c_cmd(bus, MOTOR_CTRL_ADDR, motor_lib.POLL_CMD)
                    available = list(resp)[1] == ord('D')
                    if available:
                        if LANE_EDGE_FLAG and not TURN_ACTIVE_FLAG:
                            if dist_left < SIDE_PROX_THRESH_CM:
                                send_i2c_cmd(bus, MOTOR_CTRL_ADDR, motor_lib.RIGHT_90_CMD)
                                turn_timestamp = time.time()
                                TURN_ACTIVE_FLAG = True
                            elif dist_right < SIDE_PROX_THRESH_CM:
                                send_i2c_cmd(bus, MOTOR_CTRL_ADDR, motor_lib.LEFT_90_CMD)
                                turn_timestamp = time.time()
                                TURN_ACTIVE_FLAG = True
                            else:
                                print('E: RGB active, but no clear turn condition!')
                                GOOD_TO_GO = False
                        elif RGB_DATA_GOOD_FLAG:
                            if list(resp)[0] != ord('F'):
                                send_i2c_cmd(bus, MOTOR_CTRL_ADDR, motor_lib.FORWARD_CMD)
                                print('I: starting forward again')
                        else:
                            print('I: want forward but RGB data bad')
                    elif list(resp)[0] == ord('!'):
                        print(f'E: Sec error: {list(resp)}')
                        send_i2c_cmd(bus, MOTOR_CTRL_ADDR, motor_lib.CLEAR_ERROR_CMD)
                        print('I: Clearing sec error')


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
