import RPi.GPIO as GPIO
import time
import numpy as np

class Robot:
    def __init__(self, in1, in2, ena, in3, in4, enb, wheel_diameter=5.5, wheel_base=10, frequency=1000):
        # Motor 1 Pins
        self.IN1 = in1
        self.IN2 = in2
        self.ENA = ena

        # Motor 2 Pins
        self.IN3 = in3
        self.IN4 = in4
        self.ENB = enb

        # Wheel specifications
        self.wheel_diameter = wheel_diameter  # in centimeters
        self.wheel_base = wheel_base  # distance between wheels in centimeters

        # Setup GPIO for motors
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.IN1, GPIO.OUT)
        GPIO.setup(self.IN2, GPIO.OUT)
        GPIO.setup(self.ENA, GPIO.OUT)
        GPIO.setup(self.IN3, GPIO.OUT)
        GPIO.setup(self.IN4, GPIO.OUT)
        GPIO.setup(self.ENB, GPIO.OUT)

        # Initialize PWM for motors
        self.pwmA = GPIO.PWM(self.ENA, frequency)  # PWM frequency for Motor 1
        self.pwmB = GPIO.PWM(self.ENB, frequency)  # PWM frequency for Motor 2
        self.pwmA.start(0)
        self.pwmB.start(0)

        # Robot's state [x, y, theta]
        self.state = np.array([0.0, 0.0, 0.0])

        # Path tracking list
        self.path = [self.state.copy()]

    def set_motor(self, motor, speed):
        if motor == 1:
            GPIO.output(self.IN1, GPIO.HIGH if speed > 0 else GPIO.LOW)
            GPIO.output(self.IN2, GPIO.LOW if speed > 0 else GPIO.HIGH)
            self.pwmA.ChangeDutyCycle(abs(speed))
        elif motor == 2:
            GPIO.output(self.IN3, GPIO.HIGH if speed > 0 else GPIO.LOW)
            GPIO.output(self.IN4, GPIO.LOW if speed > 0 else GPIO.HIGH)
            self.pwmB.ChangeDutyCycle(abs(speed))

    def stop(self):
        self.set_motor(1, 0)
        self.set_motor(2, 0)

    def cleanup(self):
        self.pwmA.stop()
        self.pwmB.stop()
        GPIO.cleanup()

    def __del__(self):
        self.cleanup()

    def calculate_drive_time(self, object_location, speed):
        # object_location = [x, y, theta]
        # Calculate the distance to the target
        dx = object_location[0] - self.state[0]
        dy = object_location[1] - self.state[1]
        distance = np.sqrt(dx**2 + dy**2)

        # Calculate the time needed to cover the distance at the given speed
        dt = distance / speed  # Assuming speed corresponds to linear velocity (units must be consistent)

        return dt

    def drive(self, object_location, speed):
        # Calculate the time required to drive to the target location
        dt = self.calculate_drive_time(object_location, speed)

        # Cap the time to a maximum of 5 seconds
        dt = min(dt, 5)

        # Compute the linear and angular velocities
        dx = object_location[0] - self.state[0]
        dy = object_location[1] - self.state[1]
        linear_velocity = speed  # assuming speed corresponds to linear velocity

        desired_theta = np.arctan2(dy, dx)
        dtheta = desired_theta - self.state[2]
        angular_velocity = dtheta / dt

        # Calculate wheel speeds based on linear and angular velocities
        v_left = linear_velocity - (self.wheel_base / 2) * angular_velocity
        v_right = linear_velocity + (self.wheel_base / 2) * angular_velocity

        # Convert velocities to PWM duty cycle (assuming linear relationship for simplicity)
        pwm_left = np.clip(abs(v_left), 0, 100)
        pwm_right = np.clip(abs(v_right), 0, 100)

        # Set motor speeds based on calculated PWM values
        self.set_motor(1, pwm_left if v_left >= 0 else -pwm_left)
        self.set_motor(2, pwm_right if v_right >= 0 else -pwm_right)

        # Drive for the calculated or capped duration
        time.sleep(dt)

        # Stop the motors after driving
        self.stop()

        # Update the robot's state based on the movement
        if angular_velocity == 0:
            self.state[0] += np.cos(self.state[2]) * linear_velocity * dt
            self.state[1] += np.sin(self.state[2]) * linear_velocity * dt
        else:
            th = self.state[2]
            self.state[0] += linear_velocity / angular_velocity * (np.sin(th + dt * angular_velocity) - np.sin(th))
            self.state[1] += -linear_velocity / angular_velocity * (np.cos(th + dt * angular_velocity) - np.cos(th))
            self.state[2] += dt * angular_velocity

        # Record the updated state in the path
        self.path.append(self.state.copy())

    def return_to_start_straight(self, speed):
        # Get the starting position
        start_position = self.path[0]

        # Calculate the straight-line distance and angle to the start
        dx = start_position[0] - self.state[0]
        dy = start_position[1] - self.state[1]
        distance = np.sqrt(dx**2 + dy**2)

        # Calculate the angle to the starting point
        angle_to_start = np.arctan2(dy, dx)

        # Drive directly back to the starting point while rotating
        self.drive([start_position[0], start_position[1], angle_to_start], speed)

# Example usage
if __name__ == "__main__":
    try:
        robot = Robot(17, 27, 18, 22, 23, 24)
        
        target_location = [1.0, 1.0, np.pi/4]  # Example target location
        robot.drive(target_location, speed=50)
        
        # Now return to the starting position directly while rotating
        robot.return_to_start_straight(speed=50)
        
    except KeyboardInterrupt:
        pass
    finally:
        robot.cleanup()
