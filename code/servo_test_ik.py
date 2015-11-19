import signal
import numpy as np
from pi_blaster_servos import PiBlasterServos
from servo import Servo
from kinematics.inverse.full_ik import FullIK
import cProfile as profile


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

    pbs[0] = 0
    pbs[1] = 0
    pbs[2] = 0
    pbs[3] = np.radians(-45)

    def on_terminate(signal, frame):
        pbs.turn_off()
        sys.exit()

    signal.signal(signal.SIGINT, on_terminate)

    gx_v = np.array([1, 0, 0])
    gy_v = np.array([0, 1, 0])
    gz_v = np.array([0, 0, 1])

    e_v = np.array([1, 0, 0])
    f_v = np.array([0, 1, 0])

    e_v = rotate_vector(e_v, f_v, np.radians(0))
    e_v = rotate_vector(e_v, gz_v, np.radians(45))
    f_v = rotate_vector(f_v, gz_v, np.radians(45))
    f_v = rotate_vector(f_v, e_v, np.radians(-30))

    full_ik = FullIK(platform_radius=1.42, neck_length=2.04, base_radius=3.35,
                     lower_leg_length=2.37, upper_leg_length=3.77)
    angles = full_ik.solve(e_v, f_v, 5.0)

    print angles

    if not any(np.isnan(angles)):
        pbs[0] = angles[0]
        pbs[1] = angles[1]
        pbs[2] = angles[2]
        pbs[3] = angles[3]