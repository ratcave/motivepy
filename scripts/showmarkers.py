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

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
plt.ion()
plt.show()
last_time=time.time()

while True:
    m.update_single_frame()
    ax.clear()

    update_time=time.time()
    try:
        ax.set_title("Update Rate: {0} fps".format(1./(update_time-last_time)))
    except ZeroDivisionError:
        pass
    last_time=update_time

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    if m.frame_markers():
        am=array(m.frame_markers())
        ax.scatter(am[:,0], am[:,1], am[:,2]) #list of x position of every marker, y position of every marker, z position of every marker
        plt.draw()





