import numpy as np
from dk_model import DeskbotModel
from pyswarm import pso
import cProfile as profile


dm = DeskbotModel(
    theta_b=[0 * np.pi / 180, 120 * np.pi / 180, 240 * np.pi / 180],
    r_b=[2, 2, 2],
    l=[1, 1, 1],
    m=[1, 1, 1],
    d=[1.414, 1.414, 1.414],
    h=0.5
)


def goal(x):
    dm.solve([x[0], x[1], x[2]], x[3])

    eye_vector = dm.h_v
    height = dm.h_c[2]

    print height
    # print eye_vector * 180 / np.pi

    return height - 1.5


# def con(x):
#     x1 = x[0]
#     x2 = x[1]
#     return [-(x1 + 0.25)**2 + 0.75*x2]


if __name__ == "__main__":
    np.set_printoptions(suppress=True)

    dm.solve(np.array([0, 0, 0]) * np.pi / 180, 0 * np.pi / 180)

    lb = np.array([-30, -30, -30, -120]) * np.pi / 180
    ub = np.array([30, 30, 30, 120]) * np.pi / 180

    xopt, fopt = pso(goal, lb, ub)  #, f_ieqcons=con)
    print xopt * 180 / np.pi, fopt
