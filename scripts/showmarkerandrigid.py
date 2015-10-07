__author__ = 'nico'


import motive as m
import Tkinter, tkFileDialog
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from numpy import array
import time
import itertools

root = Tkinter.Tk()
root.withdraw()
project_file_u=tkFileDialog.askopenfilename(title='Choose a project file to load: ', filetypes=[('motive projectfiles', '*.ttp')])
project_file = project_file_u.encode("ascii")
m.initialize()
m.load_project(project_file)

#TO DO: automatically change rigidbody count, and the range of unimarkers

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
plt.figtext(0.05,0.05,".", color='blue', size=50)
plt.ion()
plt.show()
color_dict={0:'red',1:'blue',2:'yellow',3:'black',4:'green',5:'magenta',6:'cyan'}
last_time=time.time()

while True:
    m.update_single_frame()

    if m.frame_markers():
        ax.clear()

        update_time=time.time()
        try:
            ax.set_title("Update Rate: {0} fps".format(1./(update_time-last_time)))
        except ZeroDivisionError:
            pass
        last_time=update_time

        rigs=m.get_rigid_bodies()

        for i in range(1,len(rigs)):
            perm_markers=list(itertools.permutations(rigs[i].point_cloud_markers))

            if len(color_dict)>=i:
                color=color_dict[i]
            else:
                color='blue'

            for p in range (0,len(perm_markers)):
                am=array(perm_markers[p])
                ax.plot(am[:,0], am[:,1], am[:,2],color) #list of x position of every marker, y position of every marker, z position of every marker in rigid body

        am=array(m.unident_markers())
        ax.scatter(am[:,0], am[:,1], am[:,2])

        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')
        plt.draw()

