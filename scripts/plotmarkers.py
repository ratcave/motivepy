__author__ = 'ratcave'


__author__ = 'nico'


import motive as m
import Tkinter, tkFileDialog
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from numpy import array
import time

root = Tkinter.Tk()
root.withdraw()
project_file_u=tkFileDialog.askopenfilename(title='Choose a project file to load: ', filetypes=[('motive projectfiles', '*.ttp')])
project_file = project_file_u.encode("ascii")
m.initialize()
m.load_project(project_file)

while m.frame_marker_count()==0:
    try:
        m.update_single_frame()
        print "Got Frame With {0} Markers".format(m.frame_marker_count())
    except RuntimeWarning:
        print "No Frame Available Error (And {0} Markers). Trying Again...".format(m.frame_marker_count())

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
plt.ion()
plt.show()
last_time=time.time()
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
am=array(m.frame_markers())
the_plot=ax.plot(am[:,0], am[:,1], am[:,2], 'bo')
plt.draw()

while True:
    try:
        if m.frame_marker_count()>0:

            update_time=time.time()
            try:
                ax.set_title("Update Rate: {0} fps".format(1./(update_time-last_time)))
            except ZeroDivisionError:
                pass
            last_time=update_time

            am=array(m.frame_markers())
            for plot in the_plot:
                plot.set_xdata(am[:,0])
                plot.set_ydata(am[:,1])

            plt.draw()

        m.update_single_frame()
        print "Got Frame With {0} Markers".format(m.frame_marker_count())
    except RuntimeWarning:
        print "No Frame Available Error (And {0} Markers). Trying Again...".format(m.frame_marker_count())

    #for i in range (0,m.rigidBody_count()):
    #    rmarkers=markers()
    #    for k in
