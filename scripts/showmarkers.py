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
    time.sleep(1)
    m.update_single_frame()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

markers=m.frame_markers()

for i in range(0,m.frame_marker_count()):
        ax.scatter(markers[3*i], markers[3*i+1], markers[3*i+2])

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

m.shutdown()
plt.show()