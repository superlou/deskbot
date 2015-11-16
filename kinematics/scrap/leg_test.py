import numpy as np
import matplotlib.pyplot as plt

m = 1
l = 1

x, z = 0.5, 0.5

a = m**2 - l**2 - x**2 - z**2
b = 2 * x * l
c = 2 * z * l

t1 = (-2 * c + np.sqrt(4 * (c**2 - a**2 + b**2))) / (2 * (a - b))
print t1
t2 = (-2 * c - np.sqrt(4 * (c**2 - a**2 + b**2))) / (2 * (a - b))
print t2

alpha1 = 2 * np.arctan(t1)
alpha2 = 2 * np.arctan(t2)
print np.degrees(alpha1), np.degrees(alpha2)

bx1, bz1 = (l * np.cos(alpha1), l * np.sin(alpha1))
bx2, bz2 = (l * np.cos(alpha2), l * np.sin(alpha2))

plt.plot([0, bx1, x], [0, bz1, z])
plt.plot([0, bx2, x], [0, bz2, z])
plt.show()
