import numpy as np
import RPi.GPIO as GPIO
import time
import os

# Motor pins (update these to match your motor driver connections)
MOTOR_LEFT_PWM = 18
MOTOR_RIGHT_PWM = 23
MOTOR_LEFT_DIR = 24
MOTOR_RIGHT_DIR = 25

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_LEFT_PWM, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT_PWM, GPIO.OUT)
GPIO.setup(MOTOR_LEFT_DIR, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT_DIR, GPIO.OUT)

# PWM Frequency
pwm_freq = 100
left_pwm = GPIO.PWM(MOTOR_LEFT_PWM, pwm_freq)
right_pwm = GPIO.PWM(MOTOR_RIGHT_PWM, pwm_freq)

left_pwm.start(0)
right_pwm.start(0)

def set_motor_speeds(left_speed, right_speed, duration):
    # Set motor directions
    GPIO.output(MOTOR_LEFT_DIR, GPIO.HIGH if left_speed >= 0 else GPIO.LOW)
    GPIO.output(MOTOR_RIGHT_DIR, GPIO.HIGH if right_speed >= 0 else GPIO.LOW)

    # Set PWM duty cycle
    left_pwm.ChangeDutyCycle(abs(left_speed))
    right_pwm.ChangeDutyCycle(abs(right_speed))

    # Wait for the specified duration
    time.sleep(duration)

    # Stop the motors
    left_pwm.ChangeDutyCycle(0)
    right_pwm.ChangeDutyCycle(0)

def calibrateWheelRadius():
    wheel_velocities_range = range(20, 80, 15)
    delta_times = []

    for wheel_vel in wheel_velocities_range:
        print(f"Driving at {wheel_vel}% speed.")
        while True:
            delta_time = input("Input the time to drive in seconds: ")
            try:
                delta_time = float(delta_time)
            except ValueError:
                print("Time must be a number.")
                continue

            # Drive the robot at the given speed for the given time
            set_motor_speeds(wheel_vel, wheel_vel, delta_time)

            uInput = input("Did the robot travel 1m?[y/N]")
            if uInput == 'y':
                delta_times.append(delta_time)
                print(f"Recording that the robot drove 1m in {delta_time:.2f} seconds at {wheel_vel} speed.\n")
                break

    scale_array = []
    for delta_time, wheel_vel in zip(delta_times, wheel_velocities_range):
        vel_world = 1 / delta_time
        scale_array.append(vel_world / wheel_vel)

    scale = sum(scale_array) / len(scale_array)
    print(f"The scale parameter is estimated as {scale:.6f}")

    return scale

def calibrateBaseline(scale):
    wheel_velocities_range = range(30, 60, 10)
    delta_times = []

    for wheel_vel in wheel_velocities_range:
        print(f"Spinning at {wheel_vel} speed.")
        while True:
            delta_time = input("Input the time to spin 360 degrees in seconds: ")
            try:
                delta_time = float(delta_time)
            except ValueError:
                print("Time must be a number.")
                continue

            # Spin the robot at the given speed for the given time
            set_motor_speeds(wheel_vel, -wheel_vel, delta_time)

            uInput = input("Did the robot spin 360 degrees?[y/N]")
            if uInput == 'y':
                delta_times.append(delta_time)
                print(f"Recording that the robot spun 360 degrees in {delta_time:.2f} seconds at {wheel_vel} speed.\n")
                break

    baseline_array = []
    for delta_time, wheel_vel in zip(delta_times, wheel_velocities_range):
        baseline_array.append(scale * wheel_vel * delta_time / np.pi)

    baseline = sum(baseline_array) / len(baseline_array)
    print(f"The baseline parameter is estimated as {baseline:.6f} m.")

    return baseline

if __name__ == "__main__":
    dataDir = "{}/param/".format(os.getcwd())

    print('Calibrating robot scale...\n')
    scale = calibrateWheelRadius()
    fileNameS = f"{dataDir}scale.txt"
    np.savetxt(fileNameS, np.array([scale]), delimiter=',')

    print('Calibrating robot baseline...\n')
    baseline = calibrateBaseline(scale)
    fileNameB = f"{dataDir}baseline.txt"
    np.savetxt(fileNameB, np.array([baseline]), delimiter=',')

    print('Finished calibration')

    # Clean up GPIO
    left_pwm.stop()
    right_pwm.stop()
    GPIO.cleanup()
