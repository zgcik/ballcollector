import RPi.GPIO as GPIO
import time
from bot.pid import PID
from camera.camera import Camera

class Pi:
    def __init__(self):
        # importing camera
        self.cam = Camera(device=2)

        # setting initial pose
        self.rob_pose = [0, 0, 0]

        # set speed
        self.speed = 50 # pwm
        self.max_rpm = 90

        # TODO: importing baseline and wheel calibration
        self.baseline = 0.028
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
        self.pid_b = PID(kp=0.15, ki=0.35, kd=0.01)

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

    def calc_speed(self, enc_count, t_int, ppr=48):
        revs = enc_count / ppr
        speed_rpm = (revs / t_int) * 60
        return speed_rpm

    def get_enc_speeds(self, t_int=1):
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

    def diff_drive(self, lin_vel, ang_vel):
        # target wheel speeds
        right_speed = (lin_vel - (ang_vel * self.baseline / 2)) / self.wheel
        left_speed = (lin_vel + (ang_vel * self.baseline / 2)) / self.wheel

        # getting speeds from encoders
        enc_right_speed, enc_left_speed = self.get_enc_speeds(t_int=1)
        
        # compute pid control outputs
        pwm_right = self.pid_a.compute(right_speed, enc_right_speed)
        pwm_left = self.pid_b.compute(left_speed, enc_left_speed)

        pwm_right = self.rpm_to_pwm(pwm_right)
        pwm_left = self.rpm_to_pwm(pwm_left)

        # clamping pid outputs
        pwm_right = max(0.0, min(100.0, pwm_right))
        pwm_left = max(0.0, min(100.0, pwm_left))

        # set motor speeds and directions
        if right_speed >= 0:
            self.set_motora_dir(1)
        else:
            self.set_motora_dir(0)
        self.set_motora_speed(abs(pwm_right))

        if left_speed >= 0:
            self.set_motorb_dir(1)
        else:
            self.set_motorb_dir(0)
        self.set_motorb_speed(abs(pwm_left))

    def drive_to_point(self, point):
        # TODO: call diff drive
        # TODO: update rob pose
        pass

    def update_rob_pose(self, rob_pose):
        self.rob_pose = rob_pose

    def stop(self):
        self.pwm_a.ChangeDutyCycle(0)
        self.pwm_b.ChangeDutyCycle(0)

    def cleanup(self):
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    pi = Pi()

    try:
        while True:
            detections = pi.cam.ball_detector(rob_pose)

            # TODO:
            if detections is not None:
                # TODO: calculate path

                # TODO: call drive to point for each point in the path

            # TODO: drive back to start

    except KeyboardInterrupt:
        pi.stop()
        pi.cleanup()