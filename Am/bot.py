import time
import numpy as np
from gpiozero_extended import Motor, PID  # Assuming gpiozero_extended contains your Motor and PID classes

# Setting general parameters
tstop = 5  # Execution duration (s)
tsample = 0.01  # Sampling period (s)
wsp = 20  # Motor speed set point (rad/s)
tau = 0.1  # Speed low-pass filter response time (s)

# Creating PID controller objects for both motors
kp = 0.15
ki = 0.35
kd = 0.01
taupid = 0.01
pid1 = PID(tsample, kp, ki, kd, umin=0, tau=taupid)
pid2 = PID(tsample, kp, ki, kd, umin=0, tau=taupid)

# Creating motor objects using GPIO pins
# Motor 1: GPIO pins 17, 27, 18; Encoders 5, 6
motor1 = Motor(enable1=18, pwm1=17, pwm2=27, encoder1=5, encoder2=6, encoderppr=48)
motor1.reset_angle()

# Motor 2: GPIO pins 22, 23, 24; Encoders 16, 26
motor2 = Motor(enable1=24, pwm1=22, pwm2=23, encoder1=16, encoder2=26, encoderppr=48)
motor2.reset_angle()

# Pre-allocating output arrays
t = []
w1, w2 = [], []
wf1, wf2 = [], []
u1, u2 = [], []

# Initializing previous and current values for both motors
ucurr1 = ucurr2 = 0
wfprev1 = wfprev2 = 0
wfcurr1 = wfcurr2 = 0
thetaprev1 = thetaprev2 = 0
tprev = tcurr = 0
tstart = time.perf_counter()

# Running execution loop
print('Running code for', tstop, 'seconds ...')
while tcurr <= tstop:
    # Pausing for `tsample` to give CPU time to process encoder signal
    time.sleep(tsample)
    # Getting current time (s)
    tcurr = time.perf_counter() - tstart

    # Getting motor shaft angular positions: I/O (data in)
    thetacurr1 = motor1.get_angle()
    thetacurr2 = motor2.get_angle()

    # Calculating motor speeds (rad/s)
    wcurr1 = np.pi / 180 * (thetacurr1 - thetaprev1) / (tcurr - tprev)
    wcurr2 = np.pi / 180 * (thetacurr2 - thetaprev2) / (tcurr - tprev)

    # Filtering motor speed signals
    wfcurr1 = tau / (tau + tsample) * wfprev1 + tsample / (tau + tsample) * wcurr1
    wfcurr2 = tau / (tau + tsample) * wfprev2 + tsample / (tau + tsample) * wcurr2
    wfprev1, wfprev2 = wfcurr1, wfcurr2

    # Calculating closed-loop outputs using PID
    ucurr1 = pid1.control(wsp, wfcurr1)
    ucurr2 = pid2.control(wsp, wfcurr2)

    # Assigning motor outputs: I/O (data out)
    motor1.set_output(ucurr1)
    motor2.set_output(ucurr2)

    # Updating output arrays
    t.append(tcurr)
    w1.append(wcurr1)
    w2.append(wcurr2)
    wf1.append(wfcurr1)
    wf2.append(wfcurr2)
    u1.append(ucurr1)
    u2.append(ucurr2)

    # Updating previous values
    thetaprev1, thetaprev2 = thetacurr1, thetacurr2
    tprev = tcurr

print('Done.')

# Stopping motors and releasing GPIO pins
motor1.set_output(0, brake=True)
motor2.set_output(0, brake=True)
del motor1, motor2
