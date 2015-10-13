__author__ = 'nico'

import motive as m
import Tkinter, tkFileDialog
from numpy import array
import time

from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as gl

root = Tkinter.Tk()
root.withdraw()
project_file_u=tkFileDialog.askopenfilename(title='Choose a project file to load: ', filetypes=[('motive projectfiles', '*.ttp')])
project_file = project_file_u.encode("ascii")
m.load_project(project_file)

m.update()

app = QtGui.QApplication([])            #create the graphing application
w = gl.GLViewWidget()                   #create widget
w.opts['distance'] = 10                 #start distance from where one looks at the plot
w.show()                                #show widget
w.setWindowTitle('markers')
sp3 = gl.GLScatterPlotItem(pos=array(m.get_frame_markers()), color=(1.0, 0.0, 0.0, 0.5), size=0.1, pxMode=False)  #pos needs array because it needs shape
w.addItem(sp3)

last_time=time.time()

def update_position():
    m.update_single_frame()
    global last_time

    if m.get_frame_markers():
        update_time=time.time()
        try:
            w.setWindowTitle('markers. Update Rate: {0} fps'.format(1./(update_time-last_time)))
        except ZeroDivisionError:
            pass

        last_time=update_time
        sp3.setData(pos=array(m.get_frame_markers()))

t = QtCore.QTimer()
t.timeout.connect(update_position)
t.start(50)

## Start Qt event loop unless running in interactive mode.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()



