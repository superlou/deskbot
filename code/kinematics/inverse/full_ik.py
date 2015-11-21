import numpy as np
from platform_ik import PlatformIK
from leg_ik import LegIK


class FullIK(object):
    def __init__(self, platform_radius, neck_length, base_radius,
                 lower_leg_length, upper_leg_length):
        self.platform_ik = PlatformIK(platform_radius, neck_length)
        self.base_radius = base_radius
        self.lower_leg_length = lower_leg_length
        self.upper_leg_length = upper_leg_length

    def solve(self, e_v, f_v, h):
        head_angle, centroid, points = self.platform_ik.solve(e_v, f_v, h)

        angles = []

        for i in range(3):
            platform_point = points[:, i]

            angle = np.radians(120.0) * i

            rot_z = np.array([
                [np.cos(angle), -np.sin(angle), 0],
                [np.sin(angle), np.cos(angle), 0],
                [0, 0, 1]
            ])

            base_point = np.dot(rot_z, np.array([self.base_radius, 0, 0]))

            angle = LegIK(base_point, platform_point, self.lower_leg_length,
                          self.upper_leg_length).solve()

            angles.append(angle)

        angles.append(head_angle)

        return angles

if __name__ == "__main__":
    np.set_printoptions(suppress=True)

    gx_v = np.array([1, 0, 0])
    gy_v = np.array([0, 1, 0])
    gz_v = np.array([0, 0, 1])

    e_v = np.array([1, 0, 0])
    f_v = np.array([0, 1, 0])

    full_ik = FullIK(platform_radius=0.5, neck_length=0.1, base_radius=1,
                     lower_leg_length=1.0, upper_leg_length=1.0)
    angles = full_ik.solve(e_v, f_v, 1)

    print angles
