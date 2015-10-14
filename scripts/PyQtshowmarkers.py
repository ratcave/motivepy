__author__ = 'nico'

import motive as m
import Tkinter, tkFileDialog
from numpy import array, linspace, vstack, zeros_like
import time
import itertools
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as gl


# Get Project
root = Tkinter.Tk()
root.withdraw()
project_file_u=tkFileDialog.askopenfilename(title='Choose a project file to load: ', filetypes=[('motive projectfiles', '*.ttp')])
project_file = project_file_u.encode("ascii")

print('Project File: {}'.format(project_file))

#project_file = 'C:/Users/ratcave/Desktop/OptiTrackPythonWrap/Cython/motivepy/Nico3.ttp'

# Load Motive Project
m.load_project(project_file)
m.update()


# Create Qt Window
app = QtGui.QApplication([])            #create the graphing application
w = gl.GLViewWidget()                   #create widget
w.opts['distance'] = 4                  #start distance from where one looks at the plot
w.setWindowTitle('markers')
w.setFixedSize(800, 800)
y_up_rotation = (90, 1, 0, 0)

# Initialize/Set Scatterplot Options
sp3 = gl.GLScatterPlotItem(pos=array([[0,0,0]]), color=(0.0, 1.0, 1.0, 0.8), size=0.025, pxMode=False)  #pos needs array because it needs shape
sp3.rotate(*y_up_rotation)
#w.setBackgroundColor('w')
sp3.setGLOptions('translucent')
w.addItem(sp3)

# Make floor rectangle
grid_points = linspace(-1, 1, 20)
points_2d =  array(list(itertools.product(grid_points, grid_points)))
points_3d =  vstack((points_2d[:,0], zeros_like(points_2d[:, 0]), points_2d[:,1])).T
plt = gl.GLScatterPlotItem(pos=points_3d, color=(0.6, 0.6, 0.6, 0.7), size=0.015, )#, width=1, antialias=True)
plt.rotate(*y_up_rotation)
w.addItem(plt)

#show widget
w.show()

last_time=time.time()
# Main Function
def update_position():
    m.update()
    global last_time
    markers = m.get_frame_markers()
    if markers:
        update_time=time.time()
        try:
            w.setWindowTitle('markers. Update Rate: {0} fps'.format(int(1./(update_time-last_time))))
        except ZeroDivisionError:
            pass

        last_time=update_time
        sp3.setData(pos=array(markers))

t = QtCore.QTimer()
t.timeout.connect(update_position)
t.start(2)

## Start Qt event loop unless running in interactive mode.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()



