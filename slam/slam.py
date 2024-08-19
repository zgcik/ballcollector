import numpy as np
import cv2

class SLAMSystem:
    def __init__(self, wheels_width, wheels_scale, camera_matrix, camera_dist, arena_size=(4.11, 6.4), start_location=(0, 0)):
        # State: [x, y, theta]'
        self.state = np.array([[start_location[0]], [start_location[1]], [0]])  # Initialize at start_location with 0 orientation
        
        # Wheel parameters
        self.wheels_width = wheels_width  # Distance between the left and right wheels
        self.wheels_scale = wheels_scale  # Scaling factor converting ticks/s to m/s

        # Camera parameters
        self.camera_matrix = camera_matrix  # Camera intrinsic matrix
        self.camera_dist = camera_dist  # Camera distortion coefficients
        
        # Arena size in meter
        self.arena_size = arena_size
        
        # Covariance matrix for state uncertainty
        self.P = np.zeros((3, 3))
    
    def update_position(self, left_speed, right_speed, dt):
        # Calculate linear and angular velocity
        linear_velocity, angular_velocity = self.convert_wheel_speeds(left_speed, right_speed)
        
        # Update the robot's state (x, y, theta) based on odometry
        if angular_velocity == 0:
            self.state[0] += np.cos(self.state[2]) * linear_velocity * dt
            self.state[1] += np.sin(self.state[2]) * linear_velocity * dt
        else:
            theta = self.state[2]
            self.state[0] += linear_velocity / angular_velocity * (np.sin(theta + dt * angular_velocity) - np.sin(theta))
            self.state[1] += -linear_velocity / angular_velocity * (np.cos(theta + dt * angular_velocity) - np.cos(theta))
            self.state[2] += dt * angular_velocity
    
    def convert_wheel_speeds(self, left_speed, right_speed):
        # Convert wheel speeds from ticks/s to m/s
        left_speed_m = left_speed * self.wheels_scale
        right_speed_m = right_speed * self.wheels_scale
        
        # Compute linear and angular velocity
        linear_velocity = (left_speed_m + right_speed_m) / 2.0
        angular_velocity = (right_speed_m - left_speed_m) / self.wheels_width
        
        return linear_velocity, angular_velocity


if __name__ == '__main__':
   pass
