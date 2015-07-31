# Likely makes use of Weierstrass Substitution
from sympy import *
from sympy.mpmath import im
import matplotlib.pyplot as plt


init_printing()
c_x, c_y, l, m = symbols('c_x c_y l m')

a, b, c, theta = symbols('a b c theta')
expr = a + b * cos(theta) + c * sin(theta)
results = solve(expr, theta)
substitutions = [
    (a, m**2 - c_x**2 - c_y**2 - l**2),
    (b, 2*l*c_x),
    (c, 2*l*c_y)
]
results = [r.subs(substitutions) for r in results]

for result in results:
    print(result)

c_points = [(0.8, 2), (1.2, 2), (1.6, 2), (2, 2), (2.4, 2),
            (2, 0.8), (2, 1.2), (2, 1.6), (2, 2.4),
            (0.8, 0.8)]
l_value = 1.5
m_value = 1.5

for c_point in c_points:
    substitutions = [
        (c_x, c_point[0]),
        (c_y, c_point[1]),
        (l, l_value),
        (m, m_value)
    ]
    solutions = [r.subs(substitutions) for r in results]
    print [r.n()*180/pi.n() for r in solutions]

    b_points = [(l_value*cos(r), l_value*sin(r)) for r in solutions]

    plt.scatter(0, 0)
    plt.scatter(c_point[0], c_point[1])

    for point in b_points:
        if not isinstance(point[0], Add) and not isinstance(point[1], Add):
            plt.plot([0, point[0], c_point[0]], [0, point[1], c_point[1]])

plt.show()
