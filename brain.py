#!/usr/bin/env python3
'''This program attempts to legally navigate the course and lock a target.

It is a fundamental behavior program that:
 - Implements the RGB, Pixy, and US sensors
 - Actuates the motors
 - Attempts to navigate the course legally (but not necessarily logically)
'''
import time
import board
import busio
import adafruit_tca9548a
import adafruit_tcs34725
import adafruit_hcsr04
from adafruit_bus_device.i2c_device import I2CDevice

import modules.pixy_smbus as pixy_lib
import modules.motor_smbus as motor_lib

#local flag to enable print statements
__DEBUG = False

#ultrasonic gpios
LEFT_US_PIN_PAIR =  (board.G19, board.G26)
RIGHT_US_PIN_PAIR = (board.G16, board.G20)

#i2c addresses
MOTOR_CTRL_ADDR = 0x69
PIXY_ADDR = 0x54
LASER_CTRL_ADDR = 0x53
MAX_I2C_MSG_BYTES = 16

#behavior metrics
SIDE_PROX_THRESH_CM = 40


if __name__ == '__main__':
    #setup
    left_us = adafruit_hcsr04.HCSR04(trigger_pin=LEFT_US_PIN_PAIR[0], echo_pin=LEFT_US_PIN_PAIR[1])
    right_us = adafruit_hcsr04.HCSR04(trigger_pin=RIGHT_US_PIN_PAIR[0], echo_pin=RIGHT_US_PIN_PAIR[1])

    GOOD_TO_GO = True
    LANE_EDGE_FLAG = False
    TARGET_IN_SIGHT = False
    TARGET_ALIGNED = False
    TURN_ACTIVE_FLAG = False
    FINAL_TARGET_SWEEP_FLAG = False
    TARGET_STATUS = 'NONE'
    BEHAVIOR = 'NAV'
    ultrasonic_timestamp = time.time()
    turn_timestamp = time.time()
    edge_timestamp = time.time()
    loop_count = 0
    x_state =''
    y_state =''
    size_state =''
    pixy_timestamp = time.time()
    read_buff = bytearray(16)

    with busio.I2C(board.SCL, board.SDA) as bus:

        mux = adafruit_tca9548a.TCA9548A(bus)
        rgb1 = adafruit_tcs34725.TCS34725(mux[2])
        pixy = I2CDevice(mux[3], PIXY_ADDR)
        motor = I2CDevice(mux[4], MOTOR_CTRL_ADDR)

        try:
            #loop
            while GOOD_TO_GO:
                if __DEBUG:
                    print(f'>>>NEW LOOP: {loop_count}')

                ##
                # RGB Sensor
                lux1 = rgb1.lux
                if __DEBUG:
                    print(f'I: Lux1 {lux1}')
                
                #if black
                if lux1 < 200:
                    LANE_EDGE_FLAG = False

                #if purple
                elif 200 < lux1 < 2000:
                    LANE_EDGE_FLAG = False

                #if yellow
                elif lux1 < 2000:
                    if not TURN_ACTIVE_FLAG:
                        with motor:
                            motor.write_then_readinto(motor_lib.STOP_CMD, read_buff)
                    if __DEBUG:
                        print('Lane edge detected!')
                    LANE_EDGE_FLAG = True

                #else bad rgb lux
                else:
                    print("E: Bad lux data")
                    break

                ##
                # Pixy
                with pixy:
                    pixy.write_then_readinto(bytearray(pixy_lib.get_blocks_cmd), read_buff)
                block_msg = pixy_lib.GetBlocksMsg(read_buff)
                if block_msg.type_code == 33 and block_msg.payload_length > 0:
                    pixy_timestamp = time.time()
                    [x_state, y_state, size_state] = pixy_lib.evaluate_cc_block(block_msg)
                    if __DEBUG:
                        print(f'I: [x, size] [{chr(x_state)}, {chr(size_state)}]')
                    if x_state == ord('G') and size_state == ord('C'):
                        TARGET_STATUS = 'LOCKED'
                        if __DEBUG:
                            print('I: TARGET LOCKED!!!')
                    elif size_state == ord('G'):
                        TARGET_STATUS = 'RANGED'
                        BEHAVIOR = 'FOLLOW'
                    elif x_state == ord('G'):
                        TARGET_STATUS = 'CENTERED'
                        BEHAVIOR = 'NAV'
                    else:
                        TARGET_STATUS = 'SIGHTED'
                        BEHAVIOR = 'NAV'

                if time.time() > pixy_timestamp+5.0:
                    TARGET_STATUS = 'NONE'

                if __DEBUG:
                    print(f'I: Target state {TARGET_STATUS}')

                ##
                # Ultrasonics
                if (LANE_EDGE_FLAG and not TURN_ACTIVE_FLAG):
                    try:
                        dist_left = left_us.distance
                    except RuntimeError:
                        dist_left = -1.0
                    try:
                        dist_right = right_us.distance
                    except RuntimeError:
                        dist_right = -1.0
                    ultrasonic_timestamp = time.time()
                    if __DEBUG:
                        print(f"Distance left: {dist_left:.1f} cm")
                        print(f"Distance right: {dist_right:.1f} cm")

                ##
                # Motor Driver
                if BEHAVIOR == 'FOLLOW':
                    if TARGET_STATUS == 'LOCKED':
                        send_i2c_cmd(bus, MOTOR_CTRL_ADDR, motor_lib.STOP_CMD)
                        print("I: Motor Logic has achieved objective")
                        time.sleep(30)
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
                elif TURN_ACTIVE_FLAG:
                    if time.time() > turn_timestamp+1.90:
                        send_i2c_cmd(bus, MOTOR_CTRL_ADDR, motor_lib.STOP_CMD)
                        TURN_ACTIVE_FLAG = False
                        if __DEBUG:
                            print('Turn complete')
                else:
                    with motor:
                        motor.write_then_readinto(motor_lib.POLL_CMD, read_buffer)
                    available = read_buffer[1] == ord('D')
                    if available:
                        if LANE_EDGE_FLAG and not TURN_ACTIVE_FLAG:
                            if 0 < dist_left < SIDE_PROX_THRESH_CM or (dist_right > SIDE_PROX_THRESH_CM and dist_left<0):
                                send_i2c_cmd(bus, MOTOR_CTRL_ADDR, motor_lib.RIGHT_90_CMD)
                                turn_timestamp = time.time()
                                TURN_ACTIVE_FLAG = True
                            elif 0 < dist_right < SIDE_PROX_THRESH_CM or (dist_left > SIDE_PROX_THRESH_CM and dist_right<0):
                                send_i2c_cmd(bus, MOTOR_CTRL_ADDR, motor_lib.LEFT_90_CMD)
                                turn_timestamp = time.time()
                                TURN_ACTIVE_FLAG = True
                            else:
                                if __DEBUG:
                                    print('E: RGB active, but no clear turn condition!')
                                GOOD_TO_GO = False
                        else:
                            if list(resp)[0] != ord('F'):
                                send_i2c_cmd(bus, MOTOR_CTRL_ADDR, motor_lib.FORWARD_CMD)
                                if __DEBUG:
                                    print('I: starting forward again')
                    elif list(resp)[0] == ord('!'):
                        if __DEBUG:
                            print(f'E: Sec error: {list(resp)}')
                        send_i2c_cmd(bus, MOTOR_CTRL_ADDR, motor_lib.CLEAR_ERROR_CMD)
                        if __DEBUG:
                            print('I: Clearing sec error')


                time.sleep(0.01)
                loop_count += 1
        except KeyboardInterrupt:
            print('T: Terminated by user')
            send_i2c_cmd(bus, MOTOR_CTRL_ADDR, motor_lib.STOP_CMD)
            print('Stop command sent')

    if not GOOD_TO_GO:
        print('I: Logical termination')

    us_lib.cleanup()
    print('Graceful exit complete')
