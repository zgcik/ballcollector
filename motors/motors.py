import RPi.GPIO as GPIO
import time

# Motor 1 pins
IN1 = 17  # GPIO pin connected to IN1 on L298N
IN2 = 27  # GPIO pin connected to IN2 on L298N
ENA = 18  # PWM pin connected to ENA on L298N

# Motor 2 pins
IN3 = 22  # GPIO pin connected to IN3 on L298N
IN4 = 23  # GPIO pin connected to IN4 on L298N
ENB = 24  # PWM pin connected to ENB on L298N

WHEEL_DIAMETER = 0.055  # (5.5 cm)
MAX_RPM = 130 # (from datasheet)

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

pwm1 = GPIO.PWM(ENA, 1000)  # PWM on ENA at 1kHz frequency
pwm2 = GPIO.PWM(ENB, 1000)  # PWM on ENB at 1kHz frequency

pwm1.start(0)  # Start PWM with 0% duty cycle
pwm2.start(0)  # Start PWM with 0% duty cycle


def calculate_duty_cycle(speed_mps):
    desired_rpm = (speed_mps * 60) / (3.14159 * WHEEL_DIAMETER)
    duty_cycle = min(max((desired_rpm / MAX_RPM) * 100, 0), 100)
    return duty_cycle

# Function to run motor
def run_motor_forward(motor, speed_mps):
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


def run_motor_backward(motor, speed_mps):
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


def stop_motor():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    pwm1.ChangeDutyCycle(0)


# Setup for the SERVO
GPIO.setmode(GPIO.BCM)
GPIO.setup(16, GPIO.OUT)

# Set up PWM on GPIO18 with 50Hz frequency
pwm = GPIO.PWM(16, 50)
pwm.start(0)

def set_servo_angle(angle):
    duty = angle / 18 + 2  # Convert angle to duty cycle
    GPIO.output(16, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(1)
    GPIO.output(16, False)
    pwm.ChangeDutyCycle(0)

# Main loop
# try:
#     while True:
#         run_motor_forward(1, 0.3)  # Run Motor 1 forward at 75% speed
#         time.sleep(5)  # Run for 5 seconds
#         run_motor_backward(1, 0.2)  # Run Motor 1 backward at 75% speed
#         time.sleep(5)
#         stop_motor()   # Stop motor

# except KeyboardInterrupt:
#     pass

# Cleanup
pwm1.stop()
pwm2.stop()
GPIO.cleanup()
