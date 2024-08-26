import time

class PID:
    def __init__(self, kp, ki, kd):
        # pid parameters
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # initialize pid vars
        self.prev_error = 0
        self.integral = 0
        self.prev_time = time.time()

    def compute(self, setpoint, actual_speed):
        current_time = time.time()
        elapsed_time = current_time - self.prev_time

        error = setpoint - actual_speed
        self.integral += error * elapsed_time
        deriv = (error - self.prev_error) / elapsed_time

        self.prev_error = error
        self.prev_time = current_time
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * deriv)

        return output


