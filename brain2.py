#!/usr/bin/env python3
''''''

import time
import RPi.GPIO as GP
from  multiprocessing import Process, Lock
from enum import Enum
from smbus2 import SMBus, i2c_msg

from modules.i2c import *

import modules.pixy_smbus as pixy_lib
import modules.ultrasonic_gpio as us_lib
import modules.motor_smbus as motor_lib

__DEBUG = False

I2C_LOCK = Lock()
US_LOCK = Lock()

class SmbusDevice(object):
    def __init__(self, bus, addr, lock):
        self.bus = bus
        self.addr = addr
        self.lock = lock


class Motor(SmbusDevice):
    class Cmds(Enum):
        forward = ord('F')
        left = ord('L')
        right = ord('R')
        stop = ord('S')
        error = ord('E')
        poll = ord('.')

    def __init__(self, bus, addr, lock):
        super().__init__(bus, addr, lock)


    def send_cmd(self, cmd, timeout_s=1.0):
        start_ts = time.time()
        while time.time() < start_ts + timeout_s:
            if self.lock.acquire(block=True, timeout=1.0):
                resp = send_i2c_cmd(self.bus, self.addr, cmd)
                self.lock.release()
                if not list(resp):
                    continue #try again
                return resp
            else:
                continue #try again
        return False


class Pixy(SmbusDevice):
    class Status(Enum):
        none = 0
        sighted = 1
        ranged = 2
        centered = 3
        locked = 4

    def __init__(self, bus, addr, lock):
        super().__init__(bus, addr, lock)
        self.blocks = []
        self.target_status = self.Status.none

    def run_pixy_poller(self, period_s=0.01, cmd=pixy_lib.get_blocks_cmd):
        self.data_ts = time.time()
        while 1:
            loop_ts = time.time()

            if self.lock.acquire(block=True, timeout=1.0):
                resp = send_i2c_cmd(self.bus, self.addr, cmd)
                self.lock.release()
                if not list(resp):
                    continue #try again
                block_msg = pixy_lib.GetBlocksMsg(resp)
                if block_msg.type_code == 33:
                    if block_msg.payload_length != 0:
                        [x_state, y_state, size_state] = pixy_lib.evaluate_cc_block(block_msg)
                        self.target_status = self.Status.sighted
                        self.data_ts = time.time()
                    else:
                        if time.time() > self.data_ts + 5.0:
                            self.target_status = self.Status.none
            else:
                continue #try again

            while time.time() <= loop_ts + period_s:
                #do nothing
                pass

    def start_sensor_in_bg(self, period_s=0.01, cmd=pixy_lib.get_blocks_cmd):
        self.proc = Process(target=self.run_pixy_poller, args=(period_s, cmd))
        self.proc.start()

    def stop_sensor_in_bg(self):
        self.proc.terminate()
        while self.proc.is_alive():
            continue #wait til term
        self.proc.close()


class UltrasonicSensor(object):
    def __init__(self, us_pair, lock):
        self.lock = lock
        self.trig_pin = us_pair[0]
        self.echo_pin = us_pair[1]
        self.pair = (self.trig_pin, self.echo_pin)
        self.setup()

    def setup(self):
        GP.setmode(GP.BCM)
        GP.setup(self.trig_pin, GP.OUT)
        GP.setup(self.echo_pin, GP.IN)
        GP.output(self.trig_pin, 0)
    
    def measure_ultrasonic_distance(self, timeout_s=1.0):
        #fixed time trigger
        GP.output(self.trig_pin, 1)
        time.sleep(0.000010)
        GP.output(self.trig_pin, 0)

        #attempt to get lock on ultrasonic iface
        if self.lock.aqcuire(block=True, timeout=1.0):
            #set a default dist of 0
            distance = 0.0

            #evaluate echo pulse start time
            start_time = time.time()
            while 1:
                if GP.input(self.echo_pin) == 1:
                    start_time = time.time()
                    break
                
                if time.time() > start_time+timeout_s:
                    distance = -1.0
                    break

            #check against timeout condition
            if not distance < 0:
                #evaluate echo pulse stop time
                stop_time = time.time()
                while 1:
                    if GP.input(self.echo_pin) == 0:
                        stop_time = time.time()
                        break

                    if time.time() > start_time+timeout_s:
                        distance = -1.0
                        break

                #one more check against timeout condition
                if not distance < 0:
                    #evaluate distance as defined by sensor docs
                    time_elapsed = stop_time - start_time
                    distance = (time_elapsed * 34300) / 2

            self.lock.release()
        else: #failed to get ultrasonic lock
            distance = -2.0

        return distance


class UltrasonicGroup(object):
    def __init__(self, left_us_pair, right_us_pair, lock):
        self.left_us_pair = UltrasonicSensor(left_us_pair, lock)
        self.right_us_pair = UltrasonicSensor(right_us_pair, lock)

        self.dist_left = -1.0
        self.dist_right = -1.0

    def us_poller(self, period_s=2.0):
        while 1:
            self.dist_left = self.left_us_pair.measure_ultrasonic_distance()
            self.dist_right = self.right_us_pair.measure_ultrasonic_distance()

            time.sleep(period_s)

    def start_sensor_in_bg(self):
        self.proc = Process(self.us_poller)
        self.proc.start()

    def stop_sensor_in_bg(self):
        self.proc.terminate()
        while self.proc.is_alive():
            continue #wait til term
        self.proc.close()


class Sense(object):
    def __init__(self):
        #declare process locks
        self.i2c_lock = Lock()
        self.pixy_lock = Lock()
        self.us_lock = Lock()
        self.rgb_lock = Lock()

        #declare processes to multiprocess
        self.pixy_proc = Process(target=self.pixy_check)
        self.us_proc = Process(target=self.us_check)
        self.rgb_proc = Process(target=self.rgb_check)

        #declare sensor data structures
        self.us_data = {dist_forward : 0.0,
                        dist_left : 0.0,
                        dist_right: 0.0}
    
    def begin(self):
        self.pixy_proc.start()
        self.us_proc.start()
        self.rgb_proc.start()
    
    def terminate(self):
        self.pixy_proc.terminate()
        self.us_proc.terminate()
        self.rgb_proc.terminate()
        time.sleep(2.0)


    def pixy_check(self):
        resp = send_i2c_cmd(bus, PIXY_ADDR, pixy_lib.get_blocks_cmd)
        if not list(resp):
            if __DEBUG:
                print('E: Pixy data failed. Keeping latest state.')
        else:
            block_msg = pixy_lib.GetBlocksMsg(resp)
            if block_msg.type_code == 33 and block_msg.payload_length != 0:
                pixy_timestamp = time.time()
                [x_state, y_state, size_state] = pixy_lib.evaluate_cc_block(block_msg)
                TARGET_STATUS = 'SIGHTED'
                # print('I: Target in sight')
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
                    BEHAVIOR = 'NAV'

        if time.time() > pixy_timestamp+5.0:
            TARGET_STATUS = 'NONE'
    
    def us_check(self):


        if __DEBUG:
            print(f'I: Target state {TARGET_STATUS}')


if __name__ == "__main__":

    with SMBus(1) as bus:
        motor = Motor(bus, MOTOR_CTRL_ADDR, I2C_LOCK)
        pixy = Pixy(bus, PIXY_ADDR, I2C_LOCK)
        us_grp = UltrasonicGroup([19,26], [16,20], US_LOCK)

        pixy.start_sensor_in_bg()
        us_grp.start_sensor_in_bg()

        try: 
            while 1:
                #poll the motor
                resp = motor.send_cmd(motor.Cmds.poll)
                if not resp:
                    print('motor timeout for poll cmd')
                    continue

                if pixy.target_status == pixy.Status.sighted:
                    resp = motor.send_cmd(motor.Cmds.stop)
                else:
                    resp = motor.send_cmd(motor.Cmds.forward)

                #debug data
                print(f'Dist Left: {us_grp.dist_left} cm')
                print(f'Dist Right: {us_grp.dist_right} cm')
                print(f'Target Status: {pixy.target_status}')

                #debug delay
                time.sleep(1.0)
        except KeyboardInterrupt:
            pixy.stop_sensor_in_bg()
            us_grp.stop_sensor_in_bg()
            print('clean exit')




