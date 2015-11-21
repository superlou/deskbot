import pyqtgraph as pg
import pyqtgraph.opengl as gl
import numpy as np
from dk_model import DeskbotModel
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.dockarea import *
from pyqtgraph.parametertree import *


def change(param, changes, deskbot_model, view):
    for param, change, data in changes:
        name = param.name()

        alpha = dm.alpha
        alpha_h = dm.alpha_h

        if name == "0":
            alpha[0] = data * np.pi / 180
        elif name == "1":
            alpha[1] = data * np.pi / 180
        elif name == "2":
            alpha[2] = data * np.pi / 180
        elif name == "h":
            alpha_h = data * np.pi / 180

        dm.solve(alpha, alpha_h)
        clear_items(view)
        create_items(view, dm.b, dm.k, dm.p, dm.p_c, dm.h_c, dm.e_p)


def clear_items(view):
    for item in view.items:
        view.removeItem(item)


def create_items(view, base_points, knee_points, platform_points,
                 platform_centroid, head_centroid, eye_point):
    zgrid = gl.GLGridItem()
    view.addItem(zgrid)

    base_points_item = gl.GLScatterPlotItem(pos=base_points.T)
    knee_points_item = gl.GLScatterPlotItem(pos=knee_points.T)
    platform_points_item = gl.GLScatterPlotItem(pos=platform_points.T)
    platform_centroid_item = gl.GLScatterPlotItem(pos=platform_centroid)
    head_centroid_item = gl.GLScatterPlotItem(pos=head_centroid)
    eye_point_item = gl.GLScatterPlotItem(pos=eye_point)

    view.addItem(base_points_item)
    view.addItem(knee_points_item)
    view.addItem(platform_points_item)
    view.addItem(platform_centroid_item)
    view.addItem(head_centroid_item)
    view.addItem(eye_point_item)

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

    dm = DeskbotModel(
        theta_b=[0 * np.pi / 180, 120 * np.pi / 180, 240 * np.pi / 180],
        r_b=[2, 2, 2],
        l=[1, 1, 1],
        m=[1, 1, 1],
        d=[1.414, 1.414, 1.414],
        h=0.5
    )

    dm.solve(np.array([0, 0, 0]) * np.pi / 180, 0 * np.pi / 180)

    pg.mkQApp()
    glViewWidget = gl.GLViewWidget()
    plotDock.addWidget(glViewWidget)

    create_items(glViewWidget, dm.b, dm.k, dm.p, dm.p_c, dm.h_c, dm.e_p)

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
    p.sigTreeStateChanged.connect(lambda p, c: change(p, c, dm, glViewWidget))
    tree.setParameters(p, showTop=False)

    treeDock.addWidget(tree)

    # Run app
    win.show()
    QtGui.QApplication.instance().exec_()
