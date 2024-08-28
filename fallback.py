import RPi.GPIO as GPIO
import time
import numpy as np
from bot.pid import PID

# from camera.camera import Camera


class Pi:
    def __init__(self):
        # importing camera
        # self.cam = Camera(device=2)

        # setting initial pose
        self.rob_pose = [0, 0, 0]

        # set speed
        self.speed = 100  # pwm
        self.max_rpm = 90
        self.linear_speed = 0.05

        # TODO: importing baseline and wheel calibration
        self.baseline = 0.0185
        self.wheel = 0.300

        # motor a: right
        self.dra_in1 = 17
        self.dra_in2 = 27
        self.dra_ena = 18

        self.dra_encodera = 5
        self.dra_encoderb = 6

        # motor b: left
        self.drb_in3 = 22
        self.drb_in4 = 23
        self.drb_enb = 19

        self.drb_encodera = 12
        self.drb_encoderb = 13

        # initialize pid objects
        self.pid_a = PID(kp=0.01, ki=0.35, kd=0.01)
        self.pid_b = PID(kp=0.01, ki=0.35, kd=0.01)

        # pwm frequency
        self.pwm_freq = 10000

        # encoder pulse counts
        self.enc_count_a = 0
        self.enc_count_b = 0

        # setting up GPIO
        self.setupGPIO()

        # pwm objects for motor speed control
        self.pwm_a = GPIO.PWM(self.dra_ena, self.pwm_freq)
        self.pwm_b = GPIO.PWM(self.drb_enb, self.pwm_freq)

        # start pwm with 0% duty cycle (motors stopped)
        self.pwm_a.start(0)
        self.pwm_b.start(0)

        self.path = []

    def setupGPIO(self):
        GPIO.setmode(GPIO.BCM)

        # motor a (right)
        GPIO.setup(self.dra_in1, GPIO.OUT)
        GPIO.setup(self.dra_in2, GPIO.OUT)
        GPIO.setup(self.dra_ena, GPIO.OUT)

        GPIO.setup(self.dra_encodera, GPIO.IN)
        GPIO.setup(self.dra_encoderb, GPIO.IN)

        # motor b (left)
        GPIO.setup(self.drb_in3, GPIO.OUT)
        GPIO.setup(self.drb_in4, GPIO.OUT)
        GPIO.setup(self.drb_enb, GPIO.OUT)

        GPIO.setup(self.drb_encodera, GPIO.IN)
        GPIO.setup(self.drb_encoderb, GPIO.IN)

        # event detection for encoders
        GPIO.add_event_detect(self.dra_encoderb, GPIO.RISING, callback=self.enc_a_cb)
        GPIO.add_event_detect(self.drb_encoderb, GPIO.RISING, callback=self.enc_b_cb)

    def enc_a_cb(self, channel):
        self.enc_count_a += 1
        # print('enca callback!')

    def enc_b_cb(self, channel):
        self.enc_count_b += 1
        # print('encb callback!')

    def rpm_to_pwm(self, rpm):
        duty_cycle = (rpm / self.max_rpm) * 100
        return duty_cycle

    def calc_speed(self, enc_count, t_int, ppr=48 * 75):
        revs = enc_count / ppr
        speed_rpm = (revs / t_int) * 60
        return speed_rpm

    def get_enc_speeds(self, t_int=0.001):
        count_a = self.enc_count_a
        count_b = self.enc_count_b

        # wait for specified time interval
        time.sleep(t_int)

        # calcualte the number of pulses in the interval
        pulses_a = self.enc_count_a - count_a
        pulses_b = self.enc_count_b - count_b

        # calculate speeds in rpm
        speed_a = self.calc_speed(pulses_a, t_int)
        speed_b = self.calc_speed(pulses_b, t_int)

        return speed_a, speed_b

    def set_motora_speed(self, pwm):
        self.pwm_a.ChangeDutyCycle(pwm)

    def set_motorb_speed(self, pwm):
        self.pwm_b.ChangeDutyCycle(pwm)

    def set_motora_dir(self, dir=1):
        if dir:
            GPIO.output(self.dra_in1, GPIO.HIGH)
            GPIO.output(self.dra_in2, GPIO.LOW)
        else:
            GPIO.output(self.dra_in1, GPIO.LOW)
            GPIO.output(self.dra_in2, GPIO.HIGH)

    def set_motorb_dir(self, dir=1):
        if dir:
            GPIO.output(self.drb_in3, GPIO.HIGH)
            GPIO.output(self.drb_in4, GPIO.LOW)
        else:
            GPIO.output(self.drb_in3, GPIO.LOW)
            GPIO.output(self.drb_in4, GPIO.HIGH)

    def update_motora_speed(self):
        speeda, _ = self.get_enc_speeds(t_int=0.1)
        pid_out = self.pid_a.compute(self.speed, speeda)
        pid_out = self.rpm_to_pwm(pid_out)
        pid_out = max(0, min(100, pid_out))
        self.set_motora_speed(pid_out)

    def update_motorb_speed(self):
        _, speedb = self.get_enc_speeds(t_int=0.1)
        pid_out = self.pid_b.compute(self.speed, speedb)
        pid_out = self.rpm_to_pwm(pid_out)
        pid_out = max(0, min(100, pid_out))
        self.set_motorb_speed(pid_out)

    def drive(self, lin_vel, ang_vel):
        right_speed = (lin_vel + (ang_vel * self.baseline / 2)) / self.wheel
        left_speed = (lin_vel + (ang_vel * self.baseline / 2)) / self.wheel
        pwm_right = self.rpm_to_pwm(right_speed)
        pwm_left = self.rpm_to_pwm(left_speed)

        pwm_right = max(0.0, min(100.0, pwm_right))
        pwm_left = max(0.0, min(100.0, pwm_left))

        self.set_motora_speed(abs(pwm_right))
        self.set_motorb_speed(abs(pwm_left))

        speeda, speedb = self.get_enc_speeds(t_int=0.001)
        pwm_right = self.pid_a.compute(right_speed, speeda)
        pwm_left = self.pid_b.compute(left_speed, speedb)
        pwm_right = max(0.0, min(100.0, pwm_right))
        pwm_left = max(0.0, min(100.0, pwm_left))

        self.set_motora_speed(abs(pwm_right))
        self.set_motorb_speed(abs(pwm_left))

    def diff_drive(self, lin_vel, ang_vel):
        right_speed = (lin_vel + (ang_vel * self.baseline / 2)) / self.wheel
        left_speed = (lin_vel - (ang_vel * self.baseline / 2)) / self.wheel

        # Determine motor directions
        self.set_motora_dir(0 if right_speed >= 0 else 1)
        self.set_motorb_dir(1 if left_speed >= 0 else 0)

        # Get current speeds from encoders
        enc_right_speed, enc_left_speed = self.get_enc_speeds(t_int=0.001)

        # Compute PID outputs for each motor
        pwm_right = self.pid_a.compute(abs(right_speed), enc_right_speed)
        pwm_left = self.pid_b.compute(abs(left_speed), enc_left_speed)

        # Convert to PWM and clamp values
        pwm_right = max(0.0, min(100.0, self.rpm_to_pwm(pwm_right)))
        pwm_left = max(0.0, min(100.0, self.rpm_to_pwm(pwm_left)))

        # Set motor speeds
        self.set_motora_speed(pwm_right)
        self.set_motorb_speed(pwm_left)

    # def diff_drive(self, lin_vel, ang_vel):
    #     # target wheel speeds
    #     right_speed = (lin_vel + (ang_vel * self.baseline / 2)) / self.wheel
    #     left_speed = (lin_vel - (ang_vel * self.baseline / 2)) / self.wheel

    #     if right_speed >= 0:
    #         self.set_motora_dir(0)
    #     else:
    #         self.set_motora_dir(1)
    #     self.set_motora_speed(abs(self.rpm_to_pwm(right_speed)))

    #     if left_speed >= 0:
    #         self.set_motorb_dir(1)
    #     else:
    #         self.set_motorb_dir(0)
    #     self.set_motorb_speed(abs(self.rpm_to_pwm(left_speed)))

    #     # getting speeds from encoders
    #     enc_right_speed, enc_left_speed = self.get_enc_speeds(t_int=0.001)

    #     # compute pid control outputs
    #     pwm_right = self.pid_a.compute(right_speed, enc_right_speed)
    #     pwm_left = self.pid_b.compute(left_speed, enc_left_speed)

    #     pwm_right = self.rpm_to_pwm(pwm_right)
    #     pwm_left = self.rpm_to_pwm(pwm_left)

    #     # clamping pid outputs
    #     pwm_right = max(0.0, min(100.0, pwm_right))
    #     pwm_left = max(0.0, min(100.0, pwm_left))

    #     # set motor speeds and directions
    #     if right_speed >= 0:
    #         self.set_motora_dir(0)
    #     else:
    #         pwm_right = self.pid_a.compute(right_speed, -enc_right_speed)
    #         self.set_motora_dir(1)
    #     self.set_motora_speed(abs(pwm_right))

    #     if left_speed >= 0:
    #         self.set_motorb_dir(1)
    #     else:
    #         pwm_left = self.pid_b.compute(left_speed, -enc_left_speed)
    #         self.set_motorb_dir(0)
    #     self.set_motorb_speed(abs(pwm_left))

    def calculate_distance(self, point):
        distance = np.sqrt(
            (point[0] - self.rob_pose[0]) ** 2 + (point[1] - self.rob_pose[1]) ** 2
        )
        return distance

    def calculate_angle(self, point):
        dx = point[0] - self.rob_pose[0]
        dy = point[0] - self.rob_pose[0]
        theta = np.arctan2(dy, dx)
        theta_normalised = (theta + np.pi) % (2 * np.pi) - np.pi
        return theta_normalised

    def drive_to_point(self, point):

        # Find the distance to the new Point:
        distance = self.calculate_distance(point)

        # Find the Angle of rotation:
        angle = self.calculate_angle(point)

        # Find the linear velocity:
        dt_linear = distance / self.linear_speed
        linear_vel = distance / dt_linear

        # Find the angular velocity:
        angular_vel = linear_vel * np.tan(angle) / self.baseline

        dt = time.time() + dt_linear
        print(1)
        while time.time() < dt:
            self.diff_drive(linear_vel, angular_vel)

        self.update_rob_pose(distance, angle)

    def update_rob_pose(self, distance, angle):
        if distance > 0:
            self.rob_pose[0] += distance * np.cos(self.rob_pose[2])
            self.rob_pose[1] += distance * np.sin(self.rob_pose[2])

        self.rob_pose[2] = (self.rob_pose[2] + angle) % (2 * np.pi)
        self.path.append(self.rob_pose.copy())

    def stop(self):
        self.pwm_a.ChangeDutyCycle(0)
        self.pwm_b.ChangeDutyCycle(0)

    def cleanup(self):
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()


if __name__ == "__main__":
    pi = Pi()
    # while True:
    try:
        # pi.diff_drive(10.0, -15.0)
        point = [0, 1]
        pi.drive_to_point(point)

        # point = [0,-0.5]

        # pi.drive_to_point(point)

    except KeyboardInterrupt:
        pi.stop()
        pi.cleanup()

    # try:
    #     while True:
    #         detections = pi.cam.ball_detector(rob_pose)

    #         # TODO:
    #         if detections is not None:
    #             # TODO: calculate path

    #             # TODO: call drive to point for each point in the path

    #         # TODO: drive back to start

    # except KeyboardInterrupt:
    #     pi.stop()
    #     pi.cleanup()