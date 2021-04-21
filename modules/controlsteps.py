# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
 
# Use this example for digital pin control of an H-bridge driver
# like a DRV8833, TB6612 or L298N.
import time
import board
import digitalio
from adafruit_motor import stepper
 
DELAY = .001
STEPS = 7000
 
# You can use any available GPIO pin on both a microcontroller and a Raspberry Pi.
# The following pins are simply a suggestion. If you use different pins, update
# the following code to use your chosen pins.
 
# To use with CircuitPython and a microcontroller:
#coils = (
 #   digitalio.DigitalInOut(board.D9),  # A1
   # digitalio.DigitalInOut(board.D10),  # A2
   # digitalio.DigitalInOut(board.D11),  # B1
    #digitalio.DigitalInOut(board.D12),  # B2
#)
 # 27 22 23 24
# To use with a Raspberry Pi:
coils = (
     digitalio.DigitalInOut(board.D27),  # A1
     digitalio.DigitalInOut(board.D23),  # A2
     digitalio.DigitalInOut(board.D24),  # B1
     digitalio.DigitalInOut(board.D22),  # B2
 )
for coil in coils:
    coil.direction = digitalio.Direction.OUTPUT
 
motor = stepper.StepperMotor(coils[0], coils[1], coils[2], coils[3], microsteps=None)


switch=digitalio.DigitalInOut(board.D10)
switch.direction = digitalio.Direction.INPUT
switch.pull = digitalio.Pull.DOWN

switch2=digitalio.DigitalInOut(board.D9)
switch2.direction = digitalio.Direction.INPUT
switch2.pull = digitalio.Pull.DOWN
try:
    while 1:
        while switch.value == True:
            #for step in range(1):
            motor.onestep(direction=stepper.BACKWARD,style=stepper.INTERLEAVE)
            time.sleep(DELAY)

        while switch2.value == True:
            #for step in range(1):
            motor.onestep(direction=stepper.FORWARD,style=stepper.INTERLEAVE)
            time.sleep(DELAY)

except KeyboardInterrupt:
    pass
motor.release()