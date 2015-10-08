__author__ = 'ratcave'

import pylab
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import proj3d
fig = pylab.figure()
ax = fig.add_subplot(111, projection = '3d')
x = y = z = [1, 2, 3]
sc = ax.scatter(x,y,z)
import time
# now try to get the display coordinates of the first point

x2, y2, _ = proj3d.proj_transform(2,2,2, ax.get_proj())

label = pylab.annotate(
    "this",
    xy = (x2, y2), xytext = (-20, 20),
    textcoords = 'offset points', ha = 'right', va = 'bottom',
    bbox = dict(boxstyle = 'round,pad=0.5', fc = 'yellow', alpha = 0.5),
    arrowprops = dict(arrowstyle = '->', connectionstyle = 'arc3,rad=0'))

def update_position(e):
    x2, y2, _ = proj3d.proj_transform(1,1,1, ax.get_proj())
    label.xy = x2,y2

    time.sleep(0.01)

    label.update_positions(fig.canvas.renderer)
    fig.canvas.draw()

fig.canvas.mpl_connect('draw_event', update_position)
#fig.canvas.mpl_connect('button_release_event', update_position)
pylab.show()