import time
import board
import busio
import adafruit_tca9548a
import adafruit_tcs34725
from adafruit_bus_device.i2c_device import I2CDevice

import modules.pixy_smbus as pixy_lib

if __name__ == "__main__":
    with busio.I2C(board.SCL, board.SDA) as bus:

        mux = adafruit_tca9548a.TCA9548A(bus)
        rgb1 = adafruit_tcs34725.TCS34725(mux[2])
        pixy = I2CDevice(mux[3], 0x54)

        read_buff = bytearray(16)
        pixy_ts = time.time()
        TARGET_STATUS = 'NONE'

        try:
            while True:
                lux1 = rgb1.lux
                print(f"RGB1 Lux: {lux1}")
                time.sleep(0.2)

                with pixy:
                    pixy.write_then_readinto(bytearray(pixy_lib.get_blocks_cmd), read_buff)
                    print('Did read pixy')
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

                print(f'Target Status: {TARGET_STATUS}')
                time.sleep(0.1)
        except KeyboardInterrupt:
            print('graceful exit')
