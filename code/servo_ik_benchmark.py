import signal
import numpy as np
from pi_blaster_servos import PiBlasterServos
from servo import Servo
from kinematics.inverse.full_ik import FullIK
from kinematics.utilities import rotate_vector
import cProfile as profile


def reset_servos(pbs):
    pbs[0] = 0
    pbs[1] = 0
    pbs[2] = 0
    pbs[3] = 0


def set_servo_angles(pbs, angles):
    if not any(np.isnan(angles)):
        pbs[0] = angles[0]
        pbs[1] = angles[1]
        pbs[2] = angles[2]
        pbs[3] = angles[3]


def solve_servos(pbs, ik, e_v, f_v, height):
    angles = full_ik.solve(e_v, f_v, height)
    set_servo_angles(pbs, angles)


def pan_global(e_v, f_v, angle):
    gz_v = np.array([0, 0, 1])
    e_v = rotate_vector(e_v, gz_v, angle)
    f_v = rotate_vector(f_v, gz_v, angle)
    return e_v, f_v


def pitch_local(e_v, f_v, angle):
    e_v = rotate_vector(e_v, f_v, angle)
    return e_v, f_v


def roll_local(e_v, f_v, angle):
    f_v = rotate_vector(f_v, e_v, angle)
    return e_v, f_v

if __name__ == "__main__":
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
        curses.endwin()
        reset_servos(pbs)
        pbs.turn_off()
        sys.exit()

    signal.signal(signal.SIGINT, on_terminate)


    full_ik = FullIK(platform_radius=1.42, neck_length=2.04, base_radius=3.35,
                     lower_leg_length=2.37, upper_leg_length=3.77)


    e_v = np.array([1, 0, 0])
    f_v = np.array([0, 1, 0])
    height = 6
    solve_servos(pbs, full_ik, e_v, f_v, height)

    t = 0

    while True:
        y_t = 0.1 * np.cos(2 * np.pi * 0.1 * t)
        z_t = 0.1 * np.sin(2 * np.pi * 0.1 * t)
        e_v = np.array([1, y_t, z_t])
        e_v = e_v / np.linalg.norm(e_v)

        solve_servos(pbs, full_ik, e_v, f_v, height)

        t += 1