__author__ = 'nico'


import motive as m
import numpy as np
import time
import itertools
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as gl


def show_viewer():

    # Create Qt Window
    app = QtGui.QApplication([])  # create the graphing application
    w = gl.GLViewWidget()  # create widget
    w.opts['distance'] = 4  # start distance from where one looks at the plot
    w.setFixedSize(1100, 800)

    # Initialize/Set unidentified_markers plot with dummy data.
    unident_markers = gl.GLScatterPlotItem(pos=np.array([[0, 0, 0]]), color=(204/255, 1, 1, 0.8), size=6)
    w.addItem(unident_markers)

    # Initialize the Rigid Body Scatterplot and assign plot color to the rigid bodies
    m.update()
    rigs = m.get_rigid_bodies()

    color_dict = {'Red': (1., 0., 0.), 'Green': (0., 1., 0.), 'Yellow': (1., 1., 0.),
                  'Mag.': (1., 0., 1.), 'Orange': (1., .4, 0.)}

    for rig, color_name in zip(rigs.values(), itertools.cycle(color_dict)):
        rig.color_name = color_name
        w.addItem(gl.GLScatterPlotItem(pos=np.array([[0, 0, 0]]), color=color_dict[color_name] + (1.,), size=8))

    # Make floor rectangle
    grid_points = np.linspace(-1, 1, 200)
    points_2d =  np.array(list(itertools.product(grid_points, grid_points)))
    points_3d =  np.insert(points_2d, 1, 0, axis=1)
    w.addItem(gl.GLScatterPlotItem(pos=points_3d, color=(0.5, 0.5, 0.5, 0.3), size=0.1))

    # Rotate Everything so Y axis is up when plotted.
    [item.rotate(90, 1, 0, 0) for item in w.items]

    # Show widget (for different backgroundcolor see PyQtshowmarkers.py)
    w.show()

    # Main Draw Loop (as generator)
    def update_generator():

        last_time= time.time()
        rig_data  = ', '.join(['{0}: {1}'.format(body.name, body.color_name) for body in rigs.values()])


        while True:
            m.update_single_frame()

            # Update Title with new FPS
            try:
                fps = round(1. / (time.time() - last_time))
                last_time = time.time()
                w.setWindowTitle('MotivePy Viewer. Rigid Bodies = {{{rigid_bodies}}}. Update Rate: {fps} fps'.format(rigid_bodies = rig_data, fps=fps))
            except ZeroDivisionError:
                pass

            # Plot
            markers = m.get_unident_markers()
            if markers:
                unident_markers.setData(pos=np.array(m.get_unident_markers()))

            for rig, scat in zip(rigs.values(), w.items[1:-1]):
                 scat.setData(pos=np.array(rig.point_cloud_markers))


            # Return Nothing
            yield


    t = QtCore.QTimer()
    t.timeout.connect(update_generator().next)
    t.start(2)

    # Start Viewer App
    QtGui.QApplication.instance().exec_()