import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
import Adafruit_TCS34725

rgb_sensor = Adafruit_TCS34725.TCS34725()

# Motor 1 pins
IN1 = 17  # GPIO pin connected to IN1 on L298N
IN2 = 27  # GPIO pin connected to IN2 on L298N
ENA = 18  # PWM pin connected to ENA on L298N

# Motor 2 pins
IN3 = 22  # GPIO pin connected to IN3 on L298N
IN4 = 23  # GPIO pin connected to IN4 on L298N
ENB = 24  # PWM pin connected to ENB on L298N

# Ultrasonic pins
IN5 = #
TRIG = #

WHEEL_DIAMETER = 0.055  # (5.5 cm)
MAX_RPM = 130 # (from datasheet)

movement_log = []

# Setup GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(IN5, GPIO.OUT)
GPIO.setup(TRIG, GPIO.OUT)

pwm1 = GPIO.PWM(ENA, 1000)  # PWM on ENA at 1kHz frequency
pwm2 = GPIO.PWM(ENB, 1000)  # PWM on ENB at 1kHz frequency

pwm1.start(0)  # Start PWM with 0% duty cycle
pwm2.start(0)  # Start PWM with 0% duty cycle

def calculate_duty_cycle(speed_mps):
    desired_rpm = (speed_mps * 60) / (3.14159 * WHEEL_DIAMETER)
    duty_cycle = min(max((desired_rpm / MAX_RPM) * 100, 0), 100)
    return duty_cycle

# Function to run motor
def motor_forward(motor, speed_mps):
    if motor == 1:
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        duty_cycle = calculate_duty_cycle(speed_mps)
        pwm1.ChangeDutyCycle(duty_cycle)
    elif motor == 2:
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        duty_cycle = calculate_duty_cycle(speed_mps)
        pwm2.ChangeDutyCycle(duty_cycle)

def motor_backward(motor, speed_mps):
    if motor == 1:
        GPIO.output(IN1, GPIO.LOW)   # Motor backward
        GPIO.output(IN2, GPIO.HIGH)
        duty_cycle = calculate_duty_cycle(speed_mps)
        pwm1.ChangeDutyCycle(duty_cycle)
    elif motor == 2:
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        duty_cycle = calculate_duty_cycle(speed_mps)
        pwm2.ChangeDutyCycle(duty_cycle)  

def motor_stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    pwm1.ChangeDutyCycle(0) 

def turn_left(duration, speed):
    motor_forward(1, speed)
    motor_backward(2, speed)
  
    time.sleep(duration)  # Adjust duration for the desired turning angle

def turn_right(duration, speed):
    motor_forward(2, speed)
    motor_backward(1, speed)

    time.sleep(duration)  # Adjust duration for the desired turning angle

# Calculate distance from object
def distance_from_object(trig, echo):

    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(echo) == 0:
        start_time = time.time()

    while GPIO.input(echo) == 1:
        stop_time = time.time()

    time_elapsed = stop_time - start_time
    distance = (time_elapsed * 34300) / 2 #cm

    return distance 

# Return centre and radius of detected tennis ball
def detect_ball(frame):

    # Convert the frame to the HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the color range for detecting the tennis ball (yellow color)
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])

    # Create a mask that only includes the yellow color
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Perform morphological operations to remove small noises and fill gaps
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

     # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        # Find the largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Get the minimum enclosing circle around the largest contour
        ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
        
        center = (int(x), int(y))
        radius = int(radius)
    
        # Only proceed if the radius meets a minimum size
        if radius > 10:
            # Draw the circle and centroid on the frame (for visualization)
            cv2.circle(frame, center, radius, (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
    
    # Return the center and radius of the detected ball
    return center, radius

def is_white_tape():
    r, g, b, c = rgb_sensor.get_raw_data()
    # Assuming white tape gives high values in all channels
    white_threshold = 200  # Adjust this threshold based on testing
    if (r > white_threshold) and (g > white_threshold) and (b > white_threshold):
        return True
    else:
        return False

def align_with_start_edge():
    
    return True

def return_to_start():
    if is_white_tape(rgb_sensor):
        # Stop the robot when the white tape is detected
        motor_stop()
        print("White tape detected! Stopping and returning to start.")
    
    time.sleep(0.1)

    align_with_start_edge()

def path


 