import numpy as np
import scipy.optimize as optimize
from dk_model import DeskbotModel


dm = DeskbotModel(
    theta_b=[0 * np.pi / 180, 120 * np.pi / 180, 240 * np.pi / 180],
    r_b=[2, 2, 2],
    l=[1, 1, 1],
    m=[1, 1, 1],
    d=[1.414, 1.414, 1.414],
    h=0.5
)


def error(x):
    dm.solve([x[0], x[1], x[2]], x[3])
    print x * 180 / np.pi
    eye_vector = dm.h_v
    height = dm.h_c[2]
    print height
    print ""

    return height - 1.5


if __name__ == "__main__":
    optimize.fmin_powell(error, [0, 0, 0, 0])
