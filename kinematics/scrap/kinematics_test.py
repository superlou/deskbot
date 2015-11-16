import numpy as np
from numpy import pi, sin, cos
import matplotlib.pyplot as plt
from sympy import Matrix
from kinematic_model import KinematicModel
from pybrain.tools.xml.networkreader import NetworkReader


def motion(v):
    rate = 1
    fs = 10

    theta_0 = 0.2 * pi/2.0 * sin(2.*pi*rate*v[0]/fs)
    theta_1 = 0.2 * pi/2.0 * sin(2.*pi*rate*v[1]/fs + pi / 3.)
    theta_2 = 0.2 * pi/2.0 * sin(2.*pi*rate*v[2]/fs + 2. * pi / 3.)

    return np.array([theta_0, theta_1, theta_2])


if __name__ == "__main__":
    l, m, s = 1, 1, 1
    d30 = 30. * np.pi / 180.
    A0 = Matrix([[0], [-2], [0]])
    A1 = Matrix([[2*cos(d30)], [2*sin(d30)], [0]])
    A2 = Matrix([[-2*cos(d30)], [2*sin(d30)], [0]])
    km = KinematicModel(A0, A1, A2, np.pi/2., np.pi+d30, -d30, l, m, s)

    t_domain = np.linspace(0, 10, 30)

    theta = np.vstack([t_domain, t_domain, t_domain]).T
    theta = np.apply_along_axis(motion, 1, theta)

    plt.subplot(211)
    plt.plot(t_domain, theta[:, 0], label="theta0")
    plt.plot(t_domain, theta[:, 1], label="theta1")
    plt.plot(t_domain, theta[:, 2], label="theta2")
    plt.legend()

    platform = np.empty([0, 3])

    info = np.array([]).reshape(0, 3)

    for i, row in enumerate(theta):
        print "Solving {0} of {1}".format(i, len(theta))
        phi = km.solve_phi(row)
        a, b, c, centroid, normal, yaw, pitch = km.state_for(row, phi)

        normal_x = normal[0]
        normal_y = normal[2]
        z = centroid[2]

        info = np.vstack([info, np.array([normal_x, normal_y, z])])

    print info

    plt.subplot(212)
    plt.plot(t_domain, info[:, 0], label="normal x")
    plt.plot(t_domain, info[:, 1], label="normal y")
    plt.plot(t_domain, info[:, 2], label="centroid z")
    plt.legend()

    # Run inverse kinematic neural network
    net = NetworkReader.readFrom('ik_net.xml')

    est = np.array([]).reshape(0, 3)

    for i, row in enumerate(info):
        est = np.vstack([est, net.activate(row)])
        print est

    plt.subplot(211)
    plt.scatter(t_domain, est[:, 0], c='b')
    plt.scatter(t_domain, est[:, 1], c='g')
    plt.scatter(t_domain, est[:, 2], c='r')

    plt.show()
