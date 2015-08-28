import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


def plot_model(a, b, c):
    for start, finish in zip(a, b):
        ax.scatter([start[0], finish[0]],
                [start[1], finish[1]],
                [start[2], finish[2]])

    for start, finish in zip(b, c):
        print b, c
        ax.scatter([start[0], finish[0]],
                [start[1], finish[1]],
                [start[2], finish[2]])

    plt.show()
