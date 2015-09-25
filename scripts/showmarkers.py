__author__ = 'nico'


import motive as m
import Tkinter, tkFileDialog
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
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
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
plt.show()

while True:
    try:
        m.update_single_frame()
        print "Got Frame With {0} Markers".format(m.frame_marker_count())
        markers=m.frame_markers()
        for i in range(0,m.frame_marker_count()):
            ax.scatter(markers[i][0], markers[i][1], markers[i][2])
    except RuntimeWarning:
        print "No Frame Available Error (And {0} Markers). Trying Again...".format(m.frame_marker_count()

    #for i in range (0,m.rigidBody_count()):
    #    rmarkers=markers()
    #    for k in








