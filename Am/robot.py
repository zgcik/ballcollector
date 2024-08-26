from gpiozero_extended import Motor
import time
import math

class Robot:
    def __init__(self, max_speed=0.21, wheel_radius=0.028, base=0.3):
        # Initialize motors with the correct GPIO pins
        self.motor1 = Motor(enable1=22, enable2=23, pwm1=12, encoder1=8, encoder2=7, encoderppr=48*75)  # Right Motor
        self.motor2 = Motor(enable1=25, enable2=24, pwm1=13, encoder1=6, encoder2=5, encoderppr=48*75)  # Left Motor

        # Robot attributes
        self.max_speed = max_speed  # Max speed in m/s
        self.wheel_radius = wheel_radius  # Wheel radius in meters
        self.base = base  # Distance between wheels in meters
        self.state = [0, 0, 0]  # [x, y, theta]
        self.path = []
        self.angular_speed = math.pi / 2 / 2.7  # Angular speed in radians per second

    def calculate_drive_time(self, distance):
        """Calculate the time needed to drive a certain distance."""
        drive_time = distance / self.max_speed
        return drive_time

    def calculate_angle_to_target(self, target_location):
        """Calculate the angle to the target location from the current state."""
        delta_x = target_location[0] - self.state[0]
        delta_y = target_location[1] - self.state[1]
        target_theta = math.atan2(delta_y, delta_x)
        return target_theta

    def update_position(self, target_location):
        """Update the robot's position."""
        self.state[0] = target_location[0]
        self.state[1] = target_location[1]
        self.path.append(self.state.copy())

    def update_heading(self, new_theta):
        """Update the robot's heading."""
        self.state[2] = new_theta

    def rotate(self, angle_diff):
        """Rotate the robot to a specific angle difference."""
        if abs(angle_diff) > 0.01:  # Threshold to avoid tiny adjustments
            rotation_speed = 0.5  # Set a speed value for rotation

            # Time to rotate by angle_diff based on angular speed
            rotation_time = abs(angle_diff) / self.angular_speed
            rotation_direction = 1 if angle_diff > 0 else -1

            # Move only one wheel to rotate the robot
            if rotation_direction > 0:
                # Rotate left by moving the right wheel forward
                self.motor1.set_output(rotation_speed)
                self.motor2.set_output(0)
            else:
                # Rotate right by moving the left wheel forward
                self.motor1.set_output(0)
                self.motor2.set_output(rotation_speed)

            time.sleep(rotation_time)
            self.motor1.set_output(0)
            self.motor2.set_output(0)

            # Update robot's heading (theta) immediately after rotation
            self.update_heading((self.state[2] + angle_diff) % (2 * math.pi))

    def drive_forward(self, distance):
        """Drive the robot forward a specific distance using encoder feedback."""
        if distance > 0.01:  # Threshold to avoid unnecessary movements
            # Calculate the required number of wheel rotations
            wheel_rotations_needed = distance / (2 * math.pi * self.wheel_radius)
            # Convert wheel rotations to encoder steps
            encoder_steps_needed = wheel_rotations_needed * self.motor1._ppr

            # Reset encoders
            self.motor1.reset_angle()
            self.motor2.reset_angle()

            # Set motors to maximum speed
            self.motor1.set_output(1)
            self.motor2.set_output(1)

            # Track the total number of encoder steps
            total_steps_motor1 = 0
            total_steps_motor2 = 0

            print(f"encoder_steps_needed: {encoder_steps_needed}")
            # Drive until the required number of encoder steps is reached
            while total_steps_motor1 < encoder_steps_needed or total_steps_motor2 < encoder_steps_needed:
                # Get current encoder angles
                angle_motor1 = self.motor1.get_angle()
                angle_motor2 = self.motor2.get_angle()

                # Convert angles to steps
                steps_motor1 = angle_motor1 * self.motor1._ppr / 360
                steps_motor2 = angle_motor2 * self.motor2._ppr / 360

                # Update total steps
                total_steps_motor1 = abs(steps_motor1)
                total_steps_motor2 = abs(steps_motor2)


                print(f"total_steps_motor1: {total_steps_motor1}")
                
                # Short sleep to prevent overloading the CPU
                time.sleep(0.01)

            # Stop motors
            self.motor1.set_output(0)
            self.motor2.set_output(0)

            # Update robot position after moving straight
            new_x = self.state[0] + distance * math.cos(self.state[2])
            new_y = self.state[1] + distance * math.sin(self.state[2])
            self.update_position([new_x, new_y])

    def drive_to_location(self, target_location):
        """Drive the robot to the new location."""
        # Calculate the distance and angle to the target location
        distance = math.sqrt((target_location[0] - self.state[0])**2 + (target_location[1] - self.state[1])**2)
        target_theta = self.calculate_angle_to_target(target_location)
        angle_diff = target_theta - self.state[2]

        # Normalize angle difference to the range [-pi, pi]
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        # Rotate to face the target
        self.rotate(angle_diff)

        # Drive straight to the target location
        self.drive_forward(distance)

    def cleanup(self):
        """Cleanup GPIO resources."""
        del self.motor1, self.motor2


if __name__ == "__main__":
    # Example usage
    robot = Robot()
    robot.drive_to_location([0.5, 0])  # Drive to (0.5 meter, 0 meter)
    robot.cleanup()
