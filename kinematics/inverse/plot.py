import pyqtgraph as pg
import pyqtgraph.opengl as gl
import numpy as np
from numpy import radians, degrees
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.dockarea import *
from pyqtgraph.parametertree import *
from ik_solver import PlatformIK


# def change(param, changes, deskbot_model, view):
#     for param, change, data in changes:
#         name = param.name()
#
#         alpha = dm.alpha
#         alpha_h = dm.alpha_h
#
#         if name == "0":
#             alpha[0] = data * np.pi / 180
#         elif name == "1":
#             alpha[1] = data * np.pi / 180
#         elif name == "2":
#             alpha[2] = data * np.pi / 180
#         elif name == "h":
#             alpha_h = data * np.pi / 180
#
#         dm.solve(alpha, alpha_h)
#         clear_items(view)
#         create_items(view, dm.b, dm.k, dm.p, dm.p_c, dm.h_c, dm.e_p)


def rotate_vector(vector, axis, angle):
    """
    Uses Rodrigues rotation formula
    axis must be a normal vector
    """
    k = axis
    v = vector
    v_rot = (v * np.cos(angle) + np.cross(k, v) * np.sin(angle) +
             k * (np.dot(k, v)) * (1 - np.cos(angle)))

    return v_rot


def clear_items(view):
    for item in view.items:
        view.removeItem(item)


def create_items(view, platform_ik):
    zgrid = gl.GLGridItem()
    view.addItem(zgrid)

    platform_points = platform_ik.p
    platform_points_item = gl.GLScatterPlotItem(pos=platform_points.T)
    view.addItem(platform_points_item)

    platform_meshdata = gl.MeshData(
        vertexes=platform_points.T,
        faces=np.array([[0, 1, 2]])
    )
    platform_item = gl.GLMeshItem(meshdata=platform_meshdata)
    view.addItem(platform_item)


if __name__ == "__main__":
    app = QtGui.QApplication([])
    win = QtGui.QMainWindow()
    area = DockArea()
    win.setCentralWidget(area)
    win.resize(800, 600)

    plotDock = Dock("Plot", size=(400, 1))
    area.addDock(plotDock)

    treeDock = Dock("Propterties", size=(200, 1))
    area.addDock(treeDock, 'left')

    # Create model of robot
    np.set_printoptions(suppress=True)

    gx_v = np.array([1, 0, 0])
    gy_v = np.array([0, 1, 0])
    gz_v = np.array([0, 0, 1])

    e_v = np.array([1, 0, 0])
    f_v = np.array([0, 1, 0])

    e_v = rotate_vector(e_v, gz_v, radians(-10))
    f_v = rotate_vector(f_v, gz_v, radians(-10))

    e_v = rotate_vector(e_v, gy_v, radians(-20))
    f_v = rotate_vector(f_v, gy_v, radians(-20))

    e_v = rotate_vector(e_v, gx_v, radians(-20))
    f_v = rotate_vector(f_v, gx_v, radians(-20))

    pik = PlatformIK(l=1)
    x = pik.solve(e_v, f_v, 1)

    pg.mkQApp()
    glViewWidget = gl.GLViewWidget()
    plotDock.addWidget(glViewWidget)

    create_items(glViewWidget, pik)

    # Create parameter tree
    tree = ParameterTree()

    params = [
        {'name': 'Servo positions', 'type': 'group', 'children': [
            {'name': '0', 'type': 'float', 'value': 0},
            {'name': '1', 'type': 'float', 'value': 0},
            {'name': '2', 'type': 'float', 'value': 0},
            {'name': 'h', 'type': 'float', 'value': 0}
        ]}
    ]

    p = Parameter.create(name='params', type='group', children=params)
    # p.sigTreeStateChanged.connect(lambda p, c: change(p, c, dm, glViewWidget))
    tree.setParameters(p, showTop=False)

    treeDock.addWidget(tree)

    # Run app
    win.show()
    QtGui.QApplication.instance().exec_()
