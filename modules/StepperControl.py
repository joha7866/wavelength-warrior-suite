# Import required libraries
import sys
import time
import RPi.GPIO as GPIO

from i2c import *
import pixy_smbus as pixy_lib

__DEBUG = False

# define variables
CHAN_LIST = [25,8,7,1] # GPIO ports to use
DELAY=.001 # delay between each sequence step

STATUS = 'SEARCHING'

SEQ = [
    [1,0,0,1],
    [1,0,0,0],
    [1,1,0,0],
    [0,1,0,0],
    [0,1,1,0],
    [0,0,1,0],
    [0,0,1,1],
    [0,0,0,1],
]
IDX = 0

def setup():
  # Use BCM GPIO references
  # instead of physical pin numbers
  GPIO.setmode(GPIO.BCM)

  # Set all pins as output
  for pin in CHAN_LIST:
    GPIO.setup(pin,GPIO.OUT)

def CW():
  global IDX
  IDX = (IDX+1)%8
  GPIO.output(CHAN_LIST, SEQ[IDX])
  time.sleep(DELAY)

def CCW():
  global IDX 
  IDX = (IDX-1)%8
  GPIO.output(CHAN_LIST, SEQ[IDX])
  time.sleep(DELAY)

if __name__ == '__main__':
    setup()
    pixy_timestamp = time.time()

    # Start main loop
    with SMBus(1) as bus:
        try:
            while True:
                #check pixy every loop
                if time.time() > pixy_timestamp+0.4:
                    resp = send_i2c_cmd(bus, PIXY_ADDR, pixy_lib.get_blocks_cmd)
                    if not list(resp):
                        if __DEBUG:
                            print('E: Pixy data failed. Keeping latest state.')
                    else:
                        pixy_timestamp = time.time()
                        block_msg = pixy_lib.GetBlocksMsg(resp)
                        if block_msg.type_code == 33 and block_msg.payload_length != 0:
                            [x_state, y_state, size_state] = pixy_lib.evaluate_cc_block(block_msg)
                            # print('I: Target in sight')
                            if x_state == ord('G'):
                                STATUS = 'FIRING'
                            else:
                                STATUS = 'TARGETING'
                        else:
                            STATUS = 'SEARCHING'

                if STATUS == 'SEARCHING':
                    CW()
                elif STATUS == 'TARGETING':
                    if x_state == ord('L'):
                        CW()
                    elif x_state == ord('R'):
                        CCW()
                elif STATUS == 'FIRING':
                    GPIO.output(CHAN_LIST, (0,0,0,0))
                else:
                    print('E: fell through logic')

        except KeyboardInterrupt:
            print('T: Terminated by user')

    GPIO.output(CHAN_LIST, (0,0,0,0))
    GPIO.cleanup()
