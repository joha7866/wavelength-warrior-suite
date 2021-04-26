#!/usr/bin/env python3
'''testing'''
import time
from smbus2 import SMBus, i2c_msg

PIXY_ADDR = 0x54
PIXY_SYNC_BYTES = 0xc1ae

USE_RED_SIGMAP = 0x01
USE_BLUE_SIGMAP = 0x02
USE_BOTH_SIGMAP = 0x03

MAP_X_LENGTH = 316
MAP_Y_LENGTH = 208
MAP_X_CENTER_TOL = 0.2
MAP_Y_CENTER_TOL = 0.2
MAP_SIZE_CLOSE_TOL = 0.9
MAP_SIZE_TOL = 0.33

X_LOWER = MAP_X_LENGTH/2-MAP_X_LENGTH*MAP_X_CENTER_TOL/2
X_UPPER = MAP_X_LENGTH/2+MAP_X_LENGTH*MAP_X_CENTER_TOL/2
Y_LOWER = MAP_Y_LENGTH/2-MAP_Y_LENGTH*MAP_Y_CENTER_TOL/2
Y_UPPER = MAP_Y_LENGTH/2+MAP_Y_LENGTH*MAP_Y_CENTER_TOL/2

version_req_cmd = [0xae, 0xc1, 0x0e, 0x00]
get_blocks_cmd = [0xae, 0xc1, 0x20, 0x02, USE_BLUE_SIGMAP, 0x01]

class I2cMsg:
    '''Generic I2cMsg class for parsing and access'''
    def __init__(self, msg):
        self.parse(list(msg))

    def parse(self, msg_bytes):
        self.sync = msg_bytes[1]*16^2 + msg_bytes[0]
        self.type_code = msg_bytes[2]
        self.payload_length = msg_bytes[3]
        self.checksum = msg_bytes[5]*16^2 + msg_bytes[4]
        self.payload = []
        try:
            for i in range(self.payload_length):
                self.payload.append(msg_bytes[i+6])
        except IndexError:
            self.payload_length -= 2

    def verify_checksum(self):
        sum = 0
        for i in range(self.payload_length):
            sum += self.payload[i]

        if sum&0xff == self.checksum or (sum+self.checksum)&0xff == self.checksum:
            return True
        else:
            return False

class GetVersionMsg(I2cMsg):
    def get_hardware_version(self):
        return self.payload[1]*16^2+self.payload[0]
    
    def get_major_firmware_version(self):
        return self.payload[2]
    
    def get_minor_firmware_version(self):
        return self.payload[3]

    def get_firmware_build(self):
        return self.payload[5]*16^2+self.payload[4]

    def get_firmware_type(self):
        out_str = ''
        for i in range(8):
            out_str = out_str+chr(self.payload[i+6])
        return out_str

class GetBlocksMsg(I2cMsg):
    '''Implementation of getBLocks(sigmap,maxBlocks) from Pixy documentation'''
    def __init__(self, msg):
        self.parse(list(msg))

        self.num_blocks = self.payload_length//14

        self.signature = self.get_signature()
        self.x = self.get_x_position()
        self.y = self.get_y_position()
        self.width = self.get_width()
        self.height = self.get_height()
        self.cc_angle = self.get_cc_angle()
        self.tracking_index = self.get_tracking_index()
        self.age = self.get_age()

    def Block(object):
        def __init__(self, payload):

    def get_signature(self):
        return self.payload[1]*16^2+self.payload[0]
    
    def get_x_position(self):
        return self.payload[3]*16^2+self.payload[2]
    
    def get_y_position(self):
        return self.payload[5]*16^2+self.payload[4]

    def get_width(self):
        return self.payload[7]*16^2+self.payload[6]

    def get_height(self):
        return self.payload[9]*16^2+self.payload[8]

    def get_cc_angle(self):
        return self.payload[11]*16^2+self.payload[10]

    def get_tracking_index(self):
        return self.payload[12]

    def get_age(self):
        return self.payload[13]




def Pixy(object):
    def __init__(self, bus):
        self.pixy = I2CDevice(self.mux[4], pixy_lib.PIXY_ADDR)
        self.x_state = None
        self.y_state = None
        self.size_state = None

    def evaluate_cc_block(self, block_msg):
        x_position = block_msg.get_x_position()
        y_position = block_msg.get_y_position()
        width = block_msg.get_width()
        height = block_msg.get_height()

        if x_position < X_LOWER:
            x_state = 'L'
        elif x_position > X_UPPER:
            x_state = 'R'
        else:
            x_state = 'G'

        if y_position < Y_LOWER:
            y_state = 'U'
        elif y_position > Y_UPPER:
            y_state = 'D'
        else:
            y_state = 'G'

        if (width > MAP_X_LENGTH*MAP_SIZE_CLOSE_TOL or
                height > MAP_Y_LENGTH*MAP_SIZE_CLOSE_TOL):
            size_state = 'C'
        elif (width > MAP_X_LENGTH*MAP_SIZE_TOL or
                height > MAP_Y_LENGTH*MAP_SIZE_TOL):
            size_state = 'G'
        else:
            size_state = 'F'

        return [ord(x_state), ord(y_state), ord(size_state)]


if __name__ == '__main__':
    GOOD_TO_GO = False
    with SMBus(1) as bus:
        try:
            try:
                write = i2c_msg.write(PIXY_ADDR, version_req_cmd)
                read = i2c_msg.read(PIXY_ADDR, 20)
                bus.i2c_rdwr(write, read)
                new_msg = GetVersionMsg(read)
                print(new_msg.payload_length)
                print(new_msg.get_firmware_type())
                GOOD_TO_GO = True
            except IOError:
                print('failed on initial version check (check connection)')

            while GOOD_TO_GO:
                try:
                    write = i2c_msg.write(PIXY_ADDR, get_blocks_cmd)
                    read = i2c_msg.read(PIXY_ADDR, 32)
                    bus.i2c_rdwr(write, read)
                    new_msg = GetBlocksMsg(read)
                    if new_msg.type_code == 33 and new_msg.payload_length != 0:
                        evaluate_cc_block(new_msg)
                    time.sleep(0.2)
                except IOError:
                    print('ioerror...')
                    time.sleep(1)

        except KeyboardInterrupt:
            print('Cleaning up and exiting')
    print('Pixy smbus debug exited gracefully')
