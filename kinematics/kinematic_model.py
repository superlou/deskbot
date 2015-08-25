from sympy import *
import numpy as np


class KinematicModel(object):
    def __init__(self, A0, A1, A2, gamma0, gamma1, gamma2, l, m, s):
        self.A0 = A0
        self.A1 = A1
        self.A2 = A2
        self.gamma0 = gamma0
        self.gamma1 = gamma1
        self.gamma2 = gamma2
        self.l = l
        self.m = m
        self.s = s

        self.theta0, self.phi0, self.C0 = self.construct_arm(
            0, self.gamma0, self.A0, self.l, self.m
        )

        self.theta1, self.phi1, self.C1 = self.construct_arm(
            1, self.gamma1, self.A1, self.l, self.m
        )

        self.theta2, self.phi2, self.C2 = self.construct_arm(
            2, self.gamma2, self.A2, self.l, self.m
        )

        self.prepare_to_solve()

    def construct_arm(self, id, gamma, A, l, m):
        theta, phi = symbols("theta{0} phi{0}".format(id))

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

        return theta, phi, z_rot * C + A

    def prepare_to_solve(self):
        leg0 = self.C1 - self.C0
        leg1 = self.C2 - self.C1
        leg2 = self.C0 - self.C2
        s = self.s

        self.dist0 = leg0[0] ** 2 + leg0[1] ** 2 + leg0[2] ** 2 - s**2
        self.dist1 = leg1[0] ** 2 + leg1[1] ** 2 + leg1[2] ** 2 - s**2
        self.dist2 = leg2[0] ** 2 + leg2[1] ** 2 + leg2[2] ** 2 - s**2

    def solve_phi(self, theta):
        substitutions = [
            (self.theta0, theta[0]),
            (self.theta1, theta[1]),
            (self.theta2, theta[2])
        ]

        dist0s = self.dist0.subs(substitutions).n()
        dist1s = self.dist1.subs(substitutions).n()
        dist2s = self.dist2.subs(substitutions).n()

        solution = nsolve(
            [dist0s, dist1s, dist2s],
            [self.phi0, self.phi1, self.phi2],
            [1.57, 1.57, 1.57]
        )

        return np.array(solution)

    def c_for(self, theta, phi):
        """Determine locations of C0, C1, and C2 given thetas and phis"""
        c0 = lambdify((self.theta0, self.phi0), self.C0)(theta[0], phi[0])
        c1 = lambdify((self.theta1, self.phi1), self.C1)(theta[1], phi[1])
        c2 = lambdify((self.theta2, self.phi2), self.C2)(theta[2], phi[2])

        return c0, c1, c2

    def check(self, c, theta, phi):
        dist01 = np.linalg.norm(c[0] - c[1])
        dist12 = np.linalg.norm(c[1] - c[2])
        dist20 = np.linalg.norm(c[2] - c[0])

        print dist01, dist12, dist20
        print np.array(theta)*180./np.pi
        print np.array(phi)*180./np.pi

if __name__ == "__main__":
    l, m, s = 1, 1, 1
    d30 = 30. * pi / 180.
    A0 = Matrix([[0], [-2], [0]])
    A1 = Matrix([[2*cos(d30)], [2*sin(d30)], [0]])
    A2 = Matrix([[-2*cos(d30)], [2*sin(d30)], [0]])
    km = KinematicModel(A0, A1, A2, pi/2., pi+d30, -d30, l, m, s)

    theta = [0, 0, 0]
    phi = km.solve_phi(theta)
    c = km.c_for(theta, phi)

    km.check(c, theta, phi)
