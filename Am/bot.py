from gpiozero_extended import Motor
import time
import math

class Robot:
    def __init__(self, wheel_radius=0.028, base=0.3):
    

        # Robot attributes
        self.wheel_radius = wheel_radius  # Wheel radius in meters
        self.baseline = base  # Distance between wheels in meters
        self.state = [0, 0, 0]  # [x, y, theta]
        self.path = []
        self.dt = 0.01
        
        #Motor Information
        self.encoder_ticks = 900
        self.pwm = 0.6
        
        #PID
        kp = 0.15
        ki = 0.35
        kd = 0.02

        self.pid_motor_1 = PIDController(kp,ki,kd,pwm=self.pwm,dt=self.dt, encoder=self.encoder_ticks)
        self.pid_motor_2 = PIDController(kp,ki,kd,pwm=self.pwm,dt=self.dt, encoder=self.encoder_ticks)

        # Initialize motors with the correct GPIO pins
        self.motor1 = Motor(enable1=22, enable2=23, pwm1=12, encoder1=8, encoder2=7, encoderppr=self.encoder_ticks)  # Right Motor
        self.motor2 = Motor(enable1=25, enable2=24, pwm1=13, encoder1=6, encoder2=5, encoderppr=self.encoder_ticks)  # Left Motor

    def calculate_wheel_rotations(self, distance):
        """Calculate the required number of wheel rotations to drive a specific distance."""
        wheel_circumference = 2 * math.pi * self.wheel_radius
        rotations = distance / wheel_circumference
        return rotations

    def calculate_encoder_steps(self, distance):
        """Convert distance to encoder steps."""
        encoder_steps = distance* self.motor1._ppr/(2*math.pi*self.wheel_radius) #Encoder ticks/meter
        return encoder_steps

    def calculate_angle_to_target(self, target_location):
        """Calculate the angle to the target location from the current state."""
        delta_x = target_location[0] - self.state[0]
        delta_y = target_location[1] - self.state[1]
        target_theta = math.atan2(delta_y, delta_x)
        return target_theta


    def update_state(self, distance=0, angle_diff=0):
        """
        Update the robot's position and heading.

        :param distance: The distance to move forward in meters.
        :param angle_diff: The angle to rotate in radians.
        """
        # Update heading
        self.state[2] = (self.state[2] + angle_diff) % (2 * math.pi)

        # Calculate new position
        if distance > 0:
            self.state[0] += distance * math.cos(self.state[2])
            self.state[1] += distance * math.sin(self.state[2])

        # Append updated state to path
        self.path.append(self.state.copy())

        print(f"Updated state: {self.state}")


    def rotate(self, angle_diff):
        """Rotate the robot by a specific angle difference using encoder feedback."""
        if abs(angle_diff) > 0.01:  # Threshold to avoid tiny adjustments
            # Calculate the arc length for one wheel to achieve the desired rotation
            arc_length = abs(angle_diff) * self.baseline
            encoder_steps_needed = self.calculate_encoder_steps(arc_length)

            # Reset encoders
            self.motor1.reset_angle()
            self.motor2.reset_angle()
            
            # Determine rotation direction and set motor output
            if angle_diff > 0:
                # Rotate left by moving the right wheel forward
                self.motor1.set_output(self.pwm)
                self.motor2.set_output(0)
            else:
                # Rotate right by moving the left wheel forward
                self.motor1.set_output(0)
                self.motor2.set_output(self.pwm)

            # Track the total number of encoder steps for the moving wheel
            total_steps_motor = 0

            # Rotate until the required number of encoder steps is reached
            while total_steps_motor < encoder_steps_needed:
                # Get the current encoder steps for the moving motor

                # Calculate delta ticks for the motor
                delta_ticks_motor = current_steps_motor - previous_steps_motor

                if angle_diff > 0:
                    current_steps_motor = abs(self.motor1._encoder.steps)
                    #PID update
                    motor_1_new_pwm = self.pid_motor_1.update(delta_ticks_motor)
                    # Set motors
                    self.motor1.set_output(motor_1_new_pwm)
                else:
                    current_steps_motor = abs(self.motor2._encoder.steps)
                    #PID update
                    motor_2_new_pwm = self.pid_motor_2.update(delta_ticks_motor)
                    # Set motors
                    self.motor2.set_output(motor_2_new_pwm)

                
                # Update previous steps
                previous_steps_motor = current_steps_motor

                # Print delta ticks for debugging
                print(f"Delta ticks Motor: {delta_ticks_motor}")

                # Update the total steps for the moving motor
                total_steps_motor = current_steps_motor

                # Wait for the next time step
                time.sleep(self.dt)
                
            # Stop motors after rotation is complete
            self.motor1.set_output(0)
            self.motor2.set_output(0)


    def drive_forward(self, distance):
        """Drive the robot forward a specific distance using encoder feedback."""
        if distance > 0.01:  # Threshold to avoid unnecessary movements
            # Calculate the required number of wheel rotations
            #wheel_rotations_needed = self.calculate_wheel_rotations(distance)
            encoder_steps_needed = self.calculate_encoder_steps(distance)

            # Reset encoders
            self.motor1.reset_angle()
            self.motor2.reset_angle()

            # Set motors
            self.motor1.set_output(self.pwm)
            self.motor2.set_output(self.pwm)

            # Track the total number of encoder steps
            total_steps_motor1 = 0
            total_steps_motor2 = 0

            # Initialize previous steps for calculating delta ticks
            previous_steps_motor1 = 0
            previous_steps_motor2 = 0

            # Drive until the required number of encoder steps is reached
            while (total_steps_motor1 < encoder_steps_needed) or (total_steps_motor2 < encoder_steps_needed):

                # Get the current encoder steps
                current_steps_motor1 = abs(self.motor1._encoder.steps)
                current_steps_motor2 = abs(self.motor2._encoder.steps)

                # Calculate delta ticks for each motor
                delta_ticks_motor1 = current_steps_motor1 - previous_steps_motor1
                delta_ticks_motor2 = current_steps_motor2 - previous_steps_motor2

                # Update previous steps
                previous_steps_motor1 = current_steps_motor1
                previous_steps_motor2 = current_steps_motor2

                #PID update
                motor_1_new_pwm = self.pid_motor_1.update(delta_ticks_motor1)
                motor_2_new_pwm = self.pid_motor_2.update(delta_ticks_motor2)

                # Set motors
                self.motor1.set_output(motor_1_new_pwm)
                self.motor2.set_output(motor_2_new_pwm)

                # Print delta ticks for debugging
                print(f"Delta ticks Motor 1: {delta_ticks_motor1}")
                print(f"Delta ticks Motor 2: {delta_ticks_motor2}")

                print(f"Actual Motor 1 PWM: {delta_ticks_motor1}")
                print(f"Actual Motor 2 PWM: {delta_ticks_motor2}")

                # Check if the required number of encoder steps is reached
                total_steps_motor1 = current_steps_motor1
                total_steps_motor2 = current_steps_motor2


                # print(f"encoder_steps_needed {encoder_steps_needed}")
                # print(f"total_steps_motor1 {total_steps_motor1}")
                # print(f"total_steps_motor2 {total_steps_motor2}")
                time.sleep(self.dt)

            # Stop motors
            self.motor1.set_output(0)
            self.motor2.set_output(0)


    def drive_to_location(self, target_location):
        """Drive the robot to the new location."""
        distance = math.sqrt((target_location[0] - self.state[0])**2 + (target_location[1]-self.state[1])**2)
        target_theta = self.calculate_angle_to_target(target_location)
        angle_diff = target_theta

        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
        
        print(f"distance {distance}")
        print(f"target_theta {target_theta}")
        print(f"angle_diff {angle_diff}")
        print(f"state {self.state}")
        self.rotate(angle_diff)
        self.drive_forward(distance)
        self.update_state(distance=distance, angle_diff=angle_diff)

    def cleanup(self):
        """Cleanup GPIO resources."""
        del self.motor1, self.motor2


class PIDController:
    def __init__(self, Kp, Ki, Kd, pwm, output_limits=(0, 1), dt=0.1, encoder=0):
        """
        Initialize the PID controller with gains, setpoint, output limits, and time step.

        Parameters:
        - Kp: Proportional gain
        - Ki: Integral gain
        - Kd: Derivative gain
        - setpoint: Desired setpoint (target RPM or ticks per minute)
        - dt: Time step in seconds between PID updates
        """

        self.rpm = 75
        self.encoder_ticks = encoder #Per Rev

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.pwm = pwm
        self.setpoint = (self.rpm * self.pwm)/(100*self.encoder_ticks) #(RPM_max * PWM)/100 *PPR
        self.output_limits = output_limits
        self.dt = dt

        # Initialize variables
        self.integral = 0
        self.previous_error = 0
        self.previous_output = 0

    def calculate_actual_tick_per_min(self,delta_ticks):
        ticks_per_min = delta_ticks*60/self.dt
        return ticks_per_min

    def calcualte_pwm(self,ticks_min):
        actual_pwm = (ticks_min * 100)/(self.encoder_ticks*self.rpm)
        return actual_pwm

    def update(self, delta_ticks):
        """
        Update the PID controller.

        Parameters:
        - actual_value: The current measured value (ticks per minute)

        Returns:
        - PID output for adjusting the PWM (e.g., duty cycle percentage)
        """

        #actual_value PWM
        actual_value = self.calculate_actual_tick_per_min(delta_ticks)

        # Calculate error
        error = self.setpoint - actual_value

        # Proportional term
        P = self.Kp * error

        # Integral term
        self.integral += error * self.dt
        I = self.Ki * self.integral

        # Derivative term
        derivative = (error - self.previous_error) / self.dt
        D = self.Kd * derivative

        # Calculate total output
        output = P + I + D

        output = self.calcualte_pwm(output)

        # Apply output limits
        output = max(self.output_limits[0], min(self.output_limits[1], output))

        # Save current error and output for next iteration
        self.previous_error = error
        self.previous_output = output

        return output

    def set_setpoint(self, new_pwm):
        """
        Set a new setpoint for the PID controller.

        Parameters:
        - setpoint: The new target setpoint (target RPM or ticks per minute)
        """
        self.setpoint = (self.rpm * new_pwm)/(100*self.encoder_ticks)

    def reset(self):
        """
        Reset the PID controller (clears integral and derivative terms).
        """
        self.integral = 0
        self.previous_error = 0
        self.previous_output = 0

if __name__ == "__main__":
    robot = Robot()
    robot.drive_to_location([0.5,0])  # Drive to (0.5 meter, 0 meter)
    robot.drive_to_location([1,0.5])
    
    robot.cleanup()
