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

# while m.frame_marker_count()==0:
#     try:
#         m.update_single_frame()
#         print "Got Frame With {0} Markers".format(m.frame_marker_count())
#     except RuntimeWarning:
#         print "No Frame Available Error (And {0} Markers). Trying Again...".format(m.frame_marker_count())

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
plt.ion()
plt.show()
last_time=time.time()

crown=m.RigidBody(0)
while True:
    try:
        if m.frame_marker_count()>0:
            crown.reset_orientation()
            ax.clear()
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            update_time=time.time()
            try:
                ax.set_title("Update Rate: {0} fps".format(1./(update_time-last_time)))
            except ZeroDivisionError:
                pass
            last_time=update_time
            am=array(m.rigidBody_markers(0))
            ax.scatter( am[:,0], am[:,1], am[:,2]) #list of x position of every marker, y position of every marker, z position of every marker
            plt.draw()


        m.update_single_frame()
        print "Got Frame With {0} Markers".format(m.frame_marker_count())
    except RuntimeWarning:
        print "No Frame Available Error (And {0} Markers). Trying Again...".format(m.frame_marker_count())

    #for i in range (0,m.rigidBody_count()):
    #    rmarkers=markers()
    #    for k in
