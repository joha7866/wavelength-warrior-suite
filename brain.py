#!/usr/bin/env python3
'''Manage Pixy, RGB Sense Arduino, and Ultrasonics,
make decisions,
then command motors'''
import time
import RPi.GPIO as GP
from smbus2 import SMBus, i2c_msg

import pixy_smbus as user_pixy

#i2c info
MOTOR_CTRL_ADDR = 0x69
RGB_SENSE_ADDR = 0x55
PIXY_ADDR = 0x54
MAX_I2C_MSG_BYTES = 16

#motor info
FORWARD_CMD = [ord('F')]
LEFT_90_CMD = [ord('L')]
RIGHT_90_CMD = [ord('R')]
STOP_CMD = [ord('S')]
CLEAR_ERROR_CMD = [ord('E')]
POLL_CMD = [ord('.')]

#ultrasonic info
LEFT_ULTRASONIC_PAIR = [5,6]
RIGHT_ULTRASONIC_PAIR = [23,24]

ULTRASONIC_MEAS_TIMEOUT_S = 0.01

SIDE_DIST_TOL_CM = 40

#pixy info
version_req_cmd = [0xae, 0xc1, 0x0e, 0x00]
get_blocks_cmd =  [0xae, 0xc1, 0x20, 0x02, 0x01, 0x01]


def setup_ultrasonic_gpio_pair(ultrasonic_gpio_pair):
    '''Runs GPIO library setup commands for a given Trig/Echo pair.'''
    GP.setup(ultrasonic_gpio_pair[0], GP.OUT)
    GP.setup(ultrasonic_gpio_pair[1], GP.IN)
    GP.output(ultrasonic_gpio_pair[0], 0)

def measure_ultrasonic_distance(ultrasonic_gpio_pair):
    '''Returns ultrasonic distance in cm or -1 if timeout occured.'''
    timeout = False
    #set trigger high
    GP.output(ultrasonic_gpio_pair[0], 1)
    # set Trigger after 0.01ms to LOW
    time.sleep(0.000010)
    GP.output(ultrasonic_gpio_pair[0], 0)

    # save start_time
    start_time = time.time()
    while 1:
        if GP.input(ultrasonic_gpio_pair[1]) == 1:
            start_time = time.time()
            break

        if time.time() > start_time+ULTRASONIC_MEAS_TIMEOUT_S:
            timeout = True
            break

    # save time of arrival
    stop_time = time.time()
    while 1:
        if GP.input(ultrasonic_gpio_pair[1]) == 0:
            stop_time = time.time()
            break

        if time.time() > start_time+ULTRASONIC_MEAS_TIMEOUT_S:
            timeout = True
            break

    if timeout:
        print('ULTRASONIC TIMEOUT: '+str(ultrasonic_gpio_pair))
        return -1

    # time difference between start and arrival
    time_elapsed = stop_time - start_time
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (time_elapsed * 34300) / 2
    return distance


def send_i2c_cmd(bus, addr, cmd):
    '''Generic function to send a pair of receive, request commands to secondaries.
        Returns request data as an i2c_msg.'''
    try:
        write = i2c_msg.write(addr, cmd)
        read = i2c_msg.read(addr, MAX_I2C_MSG_BYTES)
        bus.i2c_rdwr(write, read)
        print('Wrote: ',[chr(ch) for ch in cmd])
        print('Read:  ',[chr(ch) for ch in read])
    except IOError:
        report_ioerror(addr, cmd)
        read = []
    return read

def report_ioerror(addr, cmd):
    if addr == MOTOR_CTRL_ADDR:
        dest = 'Motor Control'
    elif addr == RGB_SENSE_ADDR:
        dest = 'RGB Sense'
    elif addr == PIXY_ADDR:
        dest = 'Pixy'
    else:
        dest = 'UNRECOGNIZED DEST'
    
    print(f'E: IOError with Dest: {dest}, Cmd: {str(cmd)}')


if __name__ == '__main__':
    #setup
    GP.setmode(GP.BCM)
    setup_ultrasonic_gpio_pair(LEFT_ULTRASONIC_PAIR)
    setup_ultrasonic_gpio_pair(RIGHT_ULTRASONIC_PAIR)
    time.sleep(2) #wait for ultrasonics to settle

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

    GP.cleanup()
    print('Graceful exit complete')
