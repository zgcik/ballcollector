import numpy as np
import RPI.GPIO as GPIO
import time

def test_pins():
    '''
    blinks an LED connected to GPIO pin 18 and reads a button state connected to GPIO pin 23
    '''
    # setting GPIO mode
    GPIO.setmode(GPIO.BCM)

    # setting up GPIO pins
    GPIO.setup(18, GPIO.OUT) # LED pin
    GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_UP) # button pin with pull-up resistor

    try:
        while True:
            # blink LED
            GPIO.output(18, GPIO.HIGH)
            time.sleep(1)
            GPIO.output(18, GPIO.LOW)
            time.sleep(1)

            # read button state
            button_state = GPIO.input(23)
            if button_state == GPIO.LOW:
                print('button pressed')
            else:
                print('button released')
            time.sleep(0.1)

    except KeyboardInterrupt:
        GPIO.cleanup