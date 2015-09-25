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
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
plt.ion()
plt.show()

while True:
    try:
        if m.frame_marker_count()>0:
            markers=[]
            markers=m.frame_markers()
            am=array(markers)
            amx=am[:,0]
            amy=am[:,1]
            amz=am[:,2]
            ax.clear()
            ax.scatter(amx, amy, amz)
            plt.draw()
        m.update_single_frame()
        print "Got Frame With {0} Markers".format(m.frame_marker_count())
    except RuntimeWarning:
        print "No Frame Available Error (And {0} Markers). Trying Again...".format(m.frame_marker_count())

    #for i in range (0,m.rigidBody_count()):
    #    rmarkers=markers()
    #    for k in
