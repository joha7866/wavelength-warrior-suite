#!/usr/bin/env python3
'''Setup and interact with Ultrasonic Sensors via 2 GPIOs.
Configure ultrasonic GPIO pin pairs as [Trig, Echo].
Configure measurement timeout.
Implement setup and measurement functions.
'''
import time
import RPi.GPIO as GP

#local ultrasonic gpios for debug
__LEFT_ULTRASONIC_PAIR = [19,26]
__RIGHT_ULTRASONIC_PAIR = [16,20]

#ultrasonic info
ULTRASONIC_MEAS_TIMEOUT_S = 0.05


def setup_ultrasonic_system(ultrasonic_pair_list):
    '''High-level setup function to configure the ultrasonic sensors'''
    GP.setmode(GP.BCM)
    for ultrasonic_pair in ultrasonic_pair_list:
        setup_ultrasonic_gpio_pair(ultrasonic_pair)
    time.sleep(2) #wait for ultrasonics to settle


def setup_ultrasonic_gpio_pair(ultrasonic_gpio_pair):
    '''Runs GPIO library setup commands for a given Trig/Echo pair.'''
    GP.setup(ultrasonic_gpio_pair[0], GP.OUT)
    GP.setup(ultrasonic_gpio_pair[1], GP.IN)
    GP.output(ultrasonic_gpio_pair[0], 0)


def measure_ultrasonic_distance(ultrasonic_gpio_pair):
    '''Returns ultrasonic distance in cm, or -1 if timeout occured.'''
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


def cleanup():
    '''Simple GPIO cleanup wrapper'''
    GP.cleanup()


if __name__ == '__main__':
    print("Running ultrasonic debug")
    setup_ultrasonic_system([__LEFT_ULTRASONIC_PAIR,__RIGHT_ULTRASONIC_PAIR])

    try:
        while 1: #main loop
            dist_left = measure_ultrasonic_distance(__LEFT_ULTRASONIC_PAIR)
            if dist_left < 0:
                print('E: Timeout on left ultrasonic sensor')
            else:
                print(f'Distance left: {dist_left:.1f} cm')

            time.sleep(0.05)

            dist_right = measure_ultrasonic_distance(__RIGHT_ULTRASONIC_PAIR)
            if dist_right < 0:
                print('E: Timeout on right ultrasonic sensor')
            else:
                print(f'Distance right: {dist_right:.1f} cm')

            time.sleep(1)

    except KeyboardInterrupt:
        print('Terminated by user')
    
    cleanup()
    print('Ultrasonic debug exited gracefully')
