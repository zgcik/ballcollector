import RPi.GPIO as GPIO
import time
import numpy as np

class Robot:
    def __init__(self, in1, in2, ena, in3, in4, enb, enc_a1, enc_b1, enc_a2, enc_b2, wheel_diameter=0.055, wheel_base=0.30, frequency=1000):
        # Motor 1 Pins (Right Motor)
        self.IN1 = in1
        self.IN2 = in2
        self.ENA = ena

        # Motor 2 Pins (Left Motor)
        self.IN3 = in3
        self.IN4 = in4
        self.ENB = enb

        # Encoder pins for the left and right wheels
        self.ENC_A1 = enc_a1  # Left wheel encoder A
        self.ENC_B1 = enc_b1  # Left wheel encoder B
        self.ENC_A2 = enc_a2  # Right wheel encoder A
        self.ENC_B2 = enc_b2  # Right wheel encoder B

        # Wheel specifications
        self.wheel_diameter = wheel_diameter  # in meters
        self.wheel_base = wheel_base  # in meters
        self.wheel_circumference = self.wheel_diameter * np.pi  # in meters

        # PID constants
        self.Kp = 1.0
        self.Ki = 0.0
        self.Kd = 0.1
        self.integral_left = 0.0
        self.integral_right = 0.0
        self.last_error_left = 0.0
        self.last_error_right = 0.0

        # Setup GPIO for motors
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.IN1, GPIO.OUT)
        GPIO.setup(self.IN2, GPIO.OUT)
        GPIO.setup(self.ENA, GPIO.OUT)
        GPIO.setup(self.IN3, GPIO.OUT)
        GPIO.setup(self.IN4, GPIO.OUT)
        GPIO.setup(self.ENB, GPIO.OUT)

        # Setup GPIO for encoders
        GPIO.setup(self.ENC_A1, GPIO.IN)
        GPIO.setup(self.ENC_B1, GPIO.IN)
        GPIO.setup(self.ENC_A2, GPIO.IN)
        GPIO.setup(self.ENC_B2, GPIO.IN)

        # Initialize PWM for motors
        self.pwmA = GPIO.PWM(self.ENA, frequency)
        self.pwmB = GPIO.PWM(self.ENB, frequency)
        self.pwmA.start(0)
        self.pwmB.start(0)

        # Initialize encoder counts
        self.encoder_count_left = 0
        self.encoder_count_right = 0

        # Attach interrupts to encoder pins
        GPIO.add_event_detect(self.ENC_A1, GPIO.RISING, callback=self.encoder_callback_left)
        GPIO.add_event_detect(self.ENC_A2, GPIO.RISING, callback=self.encoder_callback_right)

        # Robot's state [x, y, theta]
        self.state = np.array([0.0, 0.0, 0.0])

        # Path tracking list
        self.path = [self.state.copy()]

    def encoder_callback_left(self, channel):
        # Update left encoder count
        self.encoder_count_left += 1

    def encoder_callback_right(self, channel):
        # Update right encoder count
        self.encoder_count_right += 1

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

    def pid_control(self,dt):
        # Read the actual speed






        # Adjust motor speeds based on PID output
        self.set_motor(2, np.clip(output_left, 0, 100))  # Left motor
        self.set_motor(1, np.clip(output_right, 0, 100))  # Right motor

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

        # Replace time.sleep with a loop that runs for the calculated time using PID
        start_time = time.time()
        while (time.time() - start_time) < dt:
            self.pid_control(v_left, v_right, dt)
            time.sleep(0.01)  # Small delay to avoid too frequent updates

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
        robot = Robot(
            in1=17, in2=27, ena=18, 
            in3=22, in4=23, enb=24, 
            enc_a1=5, enc_b1=6, enc_a2=16, enc_b2=26,
            wheel_diameter=0.055, wheel_base=0.30
        )
        
        target_location = [1, 0, 0]  # Example target location (1 meter forward)
        robot.drive(target_location, speed=0.2)  # Drive at 0.2 m/s
        
        # Now return to the starting position directly while rotating
        # robot.return_to_start_straight(speed=0.2)
        
    except KeyboardInterrupt:
        pass
    finally:
        robot.cleanup()
