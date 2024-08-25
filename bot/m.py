from gpiozero import Motor, PWMOutputDevice, DigitalInputDevice
import time
from math import sin, cos, atan2

class Robot:
    def __init__(self, in1, in2, ena, in3, in4, enb, enc_a1, enc_b1, enc_a2, enc_b2, wheel_diameter, wheel_base):
        # Motor setup using gpiozero
        self.motor_left = Motor(forward=in1, backward=in2, pwm=True)
        self.motor_right = Motor(forward=in3, backward=in4, pwm=True)
        self.pwmA = PWMOutputDevice(ena, frequency=1000)
        self.pwmB = PWMOutputDevice(enb, frequency=1000)

        # Encoder setup using gpiozero
        self.encoder_left = DigitalInputDevice(enc_a1)
        self.encoder_right = DigitalInputDevice(enc_a2)

        # Wheel and robot specifications
        self.wheel_diameter = wheel_diameter  # in meters
        self.wheel_base = wheel_base  # distance between wheels in meters
        self.encoder_ticks_per_rev = 48

        # Initialize position
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # PID constants
        self.kp = 0.1
        self.ki = 0.01
        self.kd = 0.05
        self.last_error_left = 0
        self.last_error_right = 0
        self.integral_left = 0
        self.integral_right = 0

        # Stored PWM values
        self.pwm_left = 0
        self.pwm_right = 0

        # Encoder tick counters
        self.left_ticks = 0
        self.right_ticks = 0

        # Setup encoder event detection
        self.encoder_left.when_activated = self._left_encoder_callback
        self.encoder_right.when_activated = self._right_encoder_callback

    def _left_encoder_callback(self):
        # This function is called every time a rising edge is detected on enc_a1
        self.left_ticks += 1

    def _right_encoder_callback(self):
        # This function is called every time a rising edge is detected on enc_a2
        self.right_ticks += 1

    def calculate_distance_per_tick(self):
        # Calculate the distance each wheel travels per encoder tick
        circumference = self.wheel_diameter * 3.14159
        distance_per_tick = circumference / self.encoder_ticks_per_rev
        return distance_per_tick

    def update_position(self):
        # Update the robot's x, y, and theta based on encoder ticks
        distance_per_tick = self.calculate_distance_per_tick()
        left_distance = self.left_ticks * distance_per_tick
        right_distance = self.right_ticks * distance_per_tick

        # Update theta based on the difference in distance
        self.theta += (right_distance - left_distance) / self.wheel_base
        # Update x and y based on the average distance traveled
        distance = (left_distance + right_distance) / 2
        self.x += distance * cos(self.theta)
        self.y += distance * sin(self.theta)

        # Reset encoder counts after updating position
        self.left_ticks = 0
        self.right_ticks = 0

    def pid_control(self, target_speed_left, target_speed_right):
        # Convert target speeds (m/s) to PWM values
        target_pwm_left = self.speed_to_pwm(target_speed_left)
        target_pwm_right = self.speed_to_pwm(target_speed_right)

        # Example PID loop for left motor
        error_left = target_pwm_left - self.pwm_left
        self.integral_left += error_left
        derivative_left = error_left - self.last_error_left
        pid_output_left = self.kp * error_left + self.ki * self.integral_left + self.kd * derivative_left
        self.last_error_left = error_left

        # Example PID loop for right motor
        error_right = target_pwm_right - self.pwm_right
        self.integral_right += error_right
        derivative_right = error_right - self.last_error_right
        pid_output_right = self.kp * error_right + self.ki * self.integral_right + self.kd * derivative_right
        self.last_error_right = error_right

        # Adjust PWM values based on PID output
        self.pwm_left += pid_output_left
        self.pwm_right += pid_output_right

        # Clamp PWM values to valid range [0, 1] for gpiozero PWM output
        self.pwm_left = max(0, min(1, self.pwm_left))
        self.pwm_right = max(0, min(1, self.pwm_right))

        self.set_motor_speed(self.pwm_left, self.pwm_right)

    def speed_to_pwm(self, speed):
        # Convert speed (m/s) to a PWM value (0-1)
        max_speed = 1.0  # Example max speed of robot in m/s
        return min(1, max(0, (speed / max_speed)))

    def set_motor_speed(self, speed_left, speed_right):
        # Set motor speeds using PWM
        self.pwmA.value = speed_left
        self.pwmB.value = speed_right
        self.motor_left.forward(speed_left)
        self.motor_right.forward(speed_right)

    def drive(self, x_target, y_target, dt):
        # Calculate distance and angle to target
        dx = x_target - self.x
        dy = y_target - self.y
        distance_to_target = (dx**2 + dy**2)**0.5
        angle_to_target = atan2(dy, dx)

        # Calculate the time to drive based on desired speed
        desired_speed = 0.5  # Example: 0.5 m/s
        time_to_drive = distance_to_target / desired_speed

        # Calculate target speeds for each motor
        target_speed_left = desired_speed * (1 - (self.theta - angle_to_target) / self.wheel_base)
        target_speed_right = desired_speed * (1 + (self.theta - angle_to_target) / self.wheel_base)

        # Start driving and run PID control
        start_time = time.time()
        while time.time() - start_time < time_to_drive:
            # Update PID control
            self.pid_control(target_speed_left, target_speed_right)

            # Update position based on encoder ticks
            self.update_position()

            # Sleep for PID loop timing
            time.sleep(dt)

        # Stop motors
        self.set_motor_speed(0, 0)

# Example usage
robot = Robot(
    in1=17, in2=27, ena=18, 
    in3=22, in4=23, enb=24, 
    enc_a1=5, enc_b1=6, enc_a2=16, enc_b2=26,
    wheel_diameter=0.055, wheel_base=0.30
)
robot.drive(1.0, 0.0, 0.1)  # Drive 1 meter forward
