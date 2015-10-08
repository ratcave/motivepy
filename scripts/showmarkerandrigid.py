__author__ = 'nico'


import motive as m
import Tkinter, tkFileDialog
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import proj3d
from numpy import array
import time
import itertools

root = Tkinter.Tk()
root.withdraw()
project_file_u=tkFileDialog.askopenfilename(title='Choose a project file to load: ', filetypes=[('motive projectfiles', '*.ttp')])
project_file = project_file_u.encode("ascii")
m.load_project(project_file)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
plt.ion()
plt.show()
color_dict={0:'red',1:'green',2:'black',3:'yellow',4:'cyan',5:'magenta',6:'blue'}
last_time=time.time()

while True:
    m.update_single_frame()

    if m.get_frame_markers():
        ax.clear()

        update_time=time.time()
        try:
            ax.set_title("Update Rate: {0} fps".format(1./(update_time-last_time)))
        except ZeroDivisionError:
            pass
        last_time=update_time

        am=array(m.get_unident_markers())
        mark=ax.scatter(am[:,0], am[:,1], am[:,2], label="markers")

        rigs=m.get_rigid_bodies()

        for i in range(1,len(rigs)):
            markers=rigs[i].point_cloud_markers
            x2, y2, _ = proj3d.proj_transform(markers[0][0],markers[0][1],markers[0][2], ax.get_proj())
            plt.annotate('{0}'.format(rigs[i].name),xy=(x2,y2),xytext=(-1,1),textcoords = 'offset points', ha = 'right', va = 'bottom')

            if len(color_dict)>=i:
                color=color_dict[i]
            else:
                color='blue'

            perm_markers=list(itertools.permutations(markers))

            for p in range (0,len(perm_markers)):
                am=array(perm_markers[p])
                ax.plot(am[:,0], am[:,1], am[:,2],color) #list of x position of every marker, y position of every marker, z position of every marker in rigid body

        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')

        plt.legend(handles=[mark],loc=3)
        plt.draw()

