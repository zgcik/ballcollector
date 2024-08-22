import numpy as np
import time

from slam import SLAMSystem
from camera import Camera

class RobotController:
    def __init__(self, slam_system, camera, target_position, rgb_sensor, ultrasonic_sensor):
        self.slam_system = slam_system
        self.camera = camera
        self.rgb_sensor = rgb_sensor
        self.ultrasonic_sensor = ultrasonic_sensor
        self.target_position = np.array(target_position).reshape(2, 1)  # Target position in (x, y)
        self.start_position = self.slam_system.state[:2]  # Store the initial starting position
        self.threshold = 0.1  # Acceptable distance to target
        self.boundary_detected = False
        self.obstacle_detected = False
        self.obstacle_threshold = 0.5  # Distance in meters to consider an obstacle

    def calculate_control(self, current_position, destination_position):
        # Calculate the error vector (distance to destination)
        error_vector = destination_position - current_position[:2]

        # Calculate the angle to the destination
        destination_angle = np.arctan2(error_vector[1], error_vector[0])

        # Calculate the angle error
        angle_error = destination_angle - current_position[2]

        # Normalize the angle error
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))

        # Proportional controller for rotation and translation
        linear_velocity = 0.5 * np.linalg.norm(error_vector)
        angular_velocity = 2.0 * angle_error

        return linear_velocity, angular_velocity

    def update_position(self, left_wheel_speed, right_wheel_speed):
        # Update the SLAM system with the wheel speeds
        self.slam_system.update_state(left_wheel_speed, right_wheel_speed)

    def check_boundary(self):
        # Assuming the RGB sensor provides a method to read the current color
        r, g, b = self.rgb_sensor.read_color()

        # White color detection (assuming a simple thresholding)
        if r > 200 and g > 200 and b > 200:
            print("Boundary detected!")
            self.boundary_detected = True
            return True
        return False

    def check_for_obstacles(self):
        # Assuming the ultrasonic sensor provides a method to measure distance
        distance = self.ultrasonic_sensor.measure_distance()

        if distance < self.obstacle_threshold:
            print(f"Obstacle detected at {distance:.2f} meters!")
            self.obstacle_detected = True
            return True
        return False

    def drive_to_position(self, destination_position):
        while True:
            # Check if out of bounds
            if self.check_boundary():
                # Stop the robot and potentially reverse or turn
                # TODO: stop the robot and turn around
                self.update_position(0, 0)
                print("Reversing to avoid going out of bounds...")
                self.avoid_out_of_bounds()
                continue

            # Check for obstacles
            if self.check_for_obstacles():
                # Stop the robot and avoid the obstacle
                # TODO: stop the robot 
                self.update_position(0, 0)
                print("Avoiding obstacle...")
                self.avoid_obstacle()
                continue

            # Get the current position from the SLAM system
            current_position = self.slam_system.state

            # Calculate control signals to drive to the destination
            linear_velocity, angular_velocity = self.calculate_control(current_position, destination_position)

            # Convert the control signals into wheel speeds
            left_wheel_speed = linear_velocity - angular_velocity
            right_wheel_speed = linear_velocity + angular_velocity

            # Update the position based on the wheel speeds
            self.update_position(left_wheel_speed, right_wheel_speed)

            # Check if the robot is close enough to the destination
            distance_to_destination = np.linalg.norm(destination_position - current_position[:2])
            if distance_to_destination < self.threshold:
                print("Reached destination position.")
                break

            time.sleep(0.1)

    def avoid_out_of_bounds(self):
        # Simple reversal strategy
        time.sleep(1)  # Pause for a moment
        # Drive backwards or turn to avoid the boundary
        self.update_position(-0.2, -0.2)  # Reverse both wheels slightly
        time.sleep(1)
        self.out_of_bounds_detected = False  # Reset the out-of-bounds flag

    def avoid_obstacle(self):
        # Simple obstacle avoidance strategy
        time.sleep(1)  # Pause for a moment
        # Turn the robot to avoid the obstacle
        self.update_position(-0.2, 0.2)  # Turn in place
        time.sleep(1)
        self.obstacle_detected = False  # Reset the obstacle flag

    def perform_task_and_return(self):
        # Drive to the target position (center of the circle)
        print("Driving to the target position...")
        self.drive_to_position(self.target_position)

        # Here, you can add any task-specific logic such as locating the balls
        print("Performing task at the target position...")
        time.sleep(2)  # Simulate performing a task

        # Return to the starting position
        print("Returning to the starting position...")
        self.drive_to_position(self.start_position)

class RGBSensor:
    def __init__(self, rgb_sensor):
        self.rgb_sensor = Adafruit_TCS34725.TCS34725()
        # Initialize the sensor if needed (e.g., setting up I2C communication)
    
    def read_colour(self):
        # Replace with actual sensor reading logic
        # For demonstration, let's assume the sensor returns (r, g, b) values
        r, g, b, c = self.get_raw_data()
        
        return (r, g, b)

    def is_white(self, threshold=200):
        r, g, b = self.read_colour()
        return r > threshold and g > threshold and b > threshold
    
class UltrasonicSensor:
    def __init__(self, trigger_pin, echo_pin):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        # Setup GPIO pins for trigger and echo (for platforms like Raspberry Pi)
    
    def measure_distance(self):
        
        start_time = time.time()
        stop_time = time.time()

        time_elapsed = stop_time - start_time
        distance = (time_elapsed * 34300) / 2 #cm
        
        return distance

    def is_obstacle_near(self, threshold=0.5):
        distance = self.measure_distance()
        return distance < threshold
    
if __name__ == "__main__":
    pass 