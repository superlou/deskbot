import numpy as np
from sympy import *
from pprint import pprint
import csv
from kinematic_model import KinematicModel


if __name__ == "__main__":
    l, m, s = 1, 1, 1
    d30 = 30. * pi / 180.
    A0 = Matrix([[0], [-2], [0]])
    A1 = Matrix([[2*cos(d30)], [2*sin(d30)], [0]])
    A2 = Matrix([[-2*cos(d30)], [2*sin(d30)], [0]])
    km = KinematicModel(A0, A1, A2, pi/2., pi+d30, -d30, l, m, s)

    with open('training.csv', 'w') as training:
        writer = csv.writer(training)
        writer.writerow(['theta0', 'theta1', 'theta2', 'phi0', 'phi1', 'phi2'])

        i = 0

        while i < 100:
            try:
                theta = np.random.rand(3) - 0.5
                phi = km.solve_phi(theta)
                writer.writerow(np.hstack((theta, phi.T)))

                i = i + 1
                print("{0} training points".format(i))
            except:
                pass
