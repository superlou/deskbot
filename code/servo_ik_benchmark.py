import sys
import signal
import time
import numpy as np
from pi_blaster_servos import PiBlasterServos
from kinematics.inverse.full_ik import FullIK
from kinematics.utilities import rotate_vector
import cProfile as profile
import pyximport
pyximport.install()
from servo_fast import ServoFast as Servo

def reset_servos(pbs):
    pbs[0] = 0
    pbs[1] = 0
    pbs[2] = 0
    pbs[3] = 0


def set_servo_angles(pbs, angles):
    start = time.time()
    if not any(np.isnan(angles)):
        pbs[0] = angles[0]
        pbs[1] = angles[1]
        pbs[2] = angles[2]
        pbs[3] = angles[3]
    # print time.time() - start

def behavior(pbs, full_ik, f_v, t, freq):
    start_time = time.time()
    y_t = 0.3 * np.cos(2 * np.pi * freq * t)
    z_t = 0.3 * np.sin(2 * np.pi * freq * t)
    e_v = np.array([1, y_t, z_t])
    e_v = e_v / np.linalg.norm(e_v)

    angles = full_ik.solve(e_v, f_v, height)
    set_servo_angles(pbs, angles)

    dt = time.time() - start_time
    t += dt
    # print(dt)
    return t


if __name__ == "__main__":
    print('Starting')
    s0 = Servo(0.15, 0.065, 0.225)  # HS-311
    s1 = Servo(0.15, 0.065, 0.225)
    s2 = Servo(0.15, 0.065, 0.225)
    s3 = Servo(0.15, 0.245, 0.055)  # SG92R
    pbs = PiBlasterServos()
    pbs.add(s0, 18)
    pbs.add(s1, 23)
    pbs.add(s2, 24)
    pbs.add(s3, 25)

    reset_servos(pbs)

    def on_terminate(signal, frame):
        reset_servos(pbs)
        pbs.turn_off()
        sys.exit()

    signal.signal(signal.SIGINT, on_terminate)


    full_ik = FullIK(platform_radius=1.42, neck_length=2.04, base_radius=3.35,
                     lower_leg_length=2.37, upper_leg_length=3.77)


    e_v = np.array([1, 0, 0])
    f_v = np.array([0, 1, 0])
    height = 6
    angles = full_ik.solve(e_v, f_v, height)
    set_servo_angles(pbs, angles)

    t = 0
    dt = 0
    freq = 1.0

    while True:
        t = behavior(pbs, full_ik, f_v, t, freq)
