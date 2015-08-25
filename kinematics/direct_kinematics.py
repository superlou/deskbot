import numpy as np
from sympy import *
from pprint import pprint
import csv


def construct_c(theta, phi, gamma, A, l, m):
    C = Matrix([
        [l * cos(theta) + m * cos(phi)],
        [0],
        [l * sin(theta) + m * sin(phi)]
    ])

    z_rot = Matrix([
        [cos(gamma), -sin(gamma), 0],
        [sin(gamma), cos(gamma), 0],
        [0, 0, 1]
    ])

    return z_rot * C + A


def solve_phi(theta):
    substitutions = [
        (theta0, theta[0]),
        (theta1, theta[1]),
        (theta2, theta[2])
    ]

    dist0s = dist0.subs(substitutions).n()
    dist1s = dist1.subs(substitutions).n()
    dist2s = dist2.subs(substitutions).n()

    solution = nsolve(
        [dist0s, dist1s, dist2s],
        [phi0, phi1, phi2],
        [1.57, 1.57, 1.57]
    )

    return solution


init_printing(use_unicode=True)

l, m = 1, 1
d30 = 30. * pi / 180.

theta0, phi0, gamma0 = symbols('theta0 phi0 gamma0')
A0 = Matrix([[0], [-2], [0]])
C0 = construct_c(theta0, phi0, pi/2., A0, l, m)

theta1, phi1, gamma1 = symbols('theta1 phi1 gamma0')
A1 = Matrix([[2*cos(d30)], [2*sin(d30)], [0]])
C1 = construct_c(theta1, phi1, pi+d30, A1, l, m)

theta2, phi2, gamma2 = symbols('theta2 phi2 gamma2')
A2 = Matrix([[-2*cos(d30)], [2*sin(d30)], [0]])
C2 = construct_c(theta2, phi2, -d30, A2, l, m)

# Set up system of equations
s = 1
leg0 = C1 - C0
leg1 = C2 - C1
leg2 = C0 - C2

dist0 = leg0[0] ** 2 + leg0[1] ** 2 + leg0[2] ** 2 - s**2
dist1 = leg1[0] ** 2 + leg1[1] ** 2 + leg1[2] ** 2 - s**2
dist2 = leg2[0] ** 2 + leg2[1] ** 2 + leg2[2] ** 2 - s**2

with open('training.csv', 'w') as training:
    writer = csv.writer(training)
    writer.writerow(['theta0', 'theta1', 'theta2', 'phi0', 'phi1', 'phi2'])

    i = 0

    while i < 100:
        try:
            theta = np.random.rand(3) - 0.5
            phi = solve_phi(theta)
            writer.writerow(np.hstack((theta, phi.T)))

            i = i + 1
            print("{0} training points".format(i))
        except:
            pass
