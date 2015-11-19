import signal
import numpy as np
from pi_blaster_servos import PiBlasterServos
from servo import Servo
from kinematics.inverse.full_ik import FullIK
import curses


def rotate_vector(vector, axis, angle):
    """
    Uses Rodrigues rotation formula
    axis must be a normal vector
    """
    k = axis
    v = vector
    v_rot = (v * np.cos(angle) + np.cross(k, v) * np.sin(angle) +
             k * (np.dot(k, v)) * (1 - np.cos(angle)))

    return v_rot


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
    height = 5
    solve_servos(pbs, full_ik, e_v, f_v, height)

    stdscr = curses.initscr()
    curses.cbreak()
    stdscr.keypad(1)

    stdscr.addstr(0, 0, "Press \"q\" to quit")
    stdscr.refresh()

    key = ''
    while key != ord('q'):
        key = stdscr.getch()
        stdscr.addch(1, 0, key)
        stdscr.refresh()

        if key == ord("w"):
            height += 0.2
        elif key == ord("s"):
            height -= 0.2
        elif key == curses.KEY_LEFT:
            e_v, f_v = pan_global(e_v, f_v, np.radians(5))
        elif key == curses.KEY_RIGHT:
            e_v, f_v = pan_global(e_v, f_v, np.radians(-5))
        elif key == curses.KEY_UP:
            e_v, f_v = pitch_local(e_v, f_v, np.radians(2))
        elif key == curses.KEY_DOWN:
            e_v, f_v = pitch_local(e_v, f_v, np.radians(-2))
        elif key == ord("a"):
            e_v, f_v = roll_local(e_v, f_v, np.radians(-2))
        elif key == ord("d"):
            e_v, f_v = roll_local(e_v, f_v, np.radians(2))

        solve_servos(pbs, full_ik, e_v, f_v, height)

    curses.endwin()

