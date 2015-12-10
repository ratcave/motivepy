import motive as m
import cv

m.load_project()

cams=m.get_cams()

def give_cam():
    for cam in m.get_cams():
        if "91" in cam.name:
            return cam
cam=give_cam()
cam.exposure=33000

m.update()

import numpy as np
import cv2

cap = cv2.VideoCapture(0)

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'MJPG')
out = cv2.VideoWriter('output.avi',fourcc, 20.0, (640,480))

while(cap.isOpened()):
    ret, frame = cap.read()
    m.update()
    if ret==True:
        frame = cv2.flip(frame,0)

        # write the flipped frame
        out.write(frame)

        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

# Release everything if job is finished
cap.release()
out.release()
cv2.destroyAllWindows()











import motive as m
import cv2
import cv

m.load_project()

cams=m.get_cams()

def give_cam():
    for cam in m.get_cams():
        if "91" in cam.name:
            return cam
cam=give_cam()
cam.exposure=33000

m.update()


#CODEC = cv.CV_FOURCC('D','I','V','3') # MPEG 4.3
#CODEC = cv.CV_FOURCC('M','P','4','2') # MPEG 4.2
CODEC = cv.CV_FOURCC('M','J','P','G') # Motion Jpeg
#CODEC = cv.CV_FOURCC('U','2','6','3') # H263
#CODEC = cv.CV_FOURCC('I','2','6','3') # H263I
#CODEC = cv.CV_FOURCC('F','L','V','1') # FLV
#CODEC = cv.CV_FOURCC('P','I','M','1') # MPEG-1
#CODEC = cv.CV_FOURCC('D','I','V','X') # MPEG-4 = MPEG-1

#CODEC=-1

# Initialize the video writer to write the file
writer = cv2.VideoWriter(
    'video.avi',                        # Filename
    CODEC,                              # Codec for compression
    cam.frame_rate,                     # Frames per second
    cam.frame_resolution,               # Width / Height tuple
    False                               # Color flag
)

# Capture 50 frames and write each one to the file
for i in range(0, 1000):
    print 'frame #:', i
    m.update()
    #image=cv2.imdecode(cam.get_frame_buffer(),-1)
    writer.write(cam.get_frame_buffer())

# Release the writer
writer.release()















import motive as m
import cv2
import cv

m.load_project()

cams=m.get_cams()

def give_cam():
    for cam in m.get_cams():
        if "91" in cam.name:
            return cam
cam=give_cam()

m.update()


img1 = cv2.imread('img1.jpg')
img2 = cv2.imread('img2.jpg')
#img3 = cv2.imread('3.jpg')

height , width , layers =  img1.shape

video = cv2.VideoWriter('video.mp4',cv.CV_FOURCC('M', 'J', 'P', 'G'),1,)

video.write(img1)
video.write(img2)
#video.write(img3)

cv2.destroyAllWindows()
video.release()

























import matplotlib.animation as animation
import numpy as np
from pylab import *


dpi = 100

def ani_frame():
    fig = plt.figure()
    ax = fig.add_subplot(111)
    #ax.set_aspect('equal')
    ax.get_xaxis().set_visible(False)
    ax.get_yaxis().set_visible(False)

    im = ax.imshow(rand(100,300),cmap='gray',interpolation='nearest')
    im.set_clim([0,1])
    #fig.set_size_inches([5,5])


    tight_layout()


    def update_img(n):
        tmp = rand(100,300)
        im.set_data(tmp)
        return im

    #legend(loc=0)
    ani = animation.FuncAnimation(fig,update_img,100,interval=30)
    writer = animation.writers['ffmpeg'](fps=30)

    ani.save('short_demo.mp4',writer=writer,dpi=dpi)
    #return ani
















import numpy as np
import matplotlib
#matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.animation as manimation

FFMpegWriter = manimation.writers['ffmpeg']
metadata = dict(title='Movie Test', artist='Matplotlib',
                comment='Movie support!')
writer = FFMpegWriter(fps=15, metadata=metadata)

fig = plt.figure()
l, = plt.plot([], [], 'k-o')  #the comma means unpack the list to the right with only one element into the one element of the left

plt.xlim(-5, 5)
plt.ylim(-5, 5)

x0, y0 = 0, 0

with writer.saving(fig, "writer_test_w_o_Agg.mp4", 100):
    for i in range(100):
        x0 += 0.1 * np.random.randn()
        y0 += 0.1 * np.random.randn()
        l.set_data(x0, y0)
        writer.grab_frame()

















import motive as m
import matplotlib.animation as animation
import numpy as np
from pylab import *

m.load_project()

cams=m.get_cams()

def give_cam():
    for cam in m.get_cams():
        if "91" in cam.name:
            return cam
cam=give_cam()

m.update()

dpi = 100

def ani_frame():
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.get_xaxis().set_visible(False)
    ax.get_yaxis().set_visible(False)

    im = ax.imshow(cam.get_frame_buffer(),cmap='gray',interpolation='nearest')
    im.set_clim([0,1])

    tight_layout()

    def update_img(n):
        m.update()
        tmp = cam.get_frame_buffer()
        im.set_data(tmp)
        return im

    #legend(loc=0)
    ani = animation.FuncAnimation(fig,update_img,1000,interval=30)
    writer = animation.writers['ffmpeg'](fps=30)

    ani.save('short_demo.mp4',writer=writer,dpi=dpi)

