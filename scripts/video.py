import cv
import cv2
import time
import numpy as np
import argparse
import motive as m
import Tkinter, tkFileDialog

def gui_load_project_file():
    root = Tkinter.Tk()
    root.withdraw()
    project_file=(tkFileDialog.askopenfilename(title='Choose a project file to load: ', filetypes=[('motive projectfiles', '*.ttp')])).encode('ascii')
    root.quit()
    return project_file

def get_cam(camera_name="91"):
    for cam in m.get_cams():
        if camera_name in cam.name:
            return cam

def gui_camera_name():
    #maybe try just returning the camera_name
    root = Tkinter.Tk()

    def choose_camera(camera_name):
        #root.withdraw()
        global cam
        cam=get_cam(camera_name)
        root.quit()

    frame=Tkinter.Frame(root)
    frame.pack()

    camera_button_11000=Tkinter.Button(frame, text="Camera Prime 13 #11000", command=lambda: choose_camera('11000'))
    camera_button_11000.pack(side=Tkinter.LEFT)

    camera_button_10187 = Tkinter.Button(frame, text="Camera Prime 17W #10187", command=lambda: choose_camera('10187'))
    camera_button_10187.pack(side=Tkinter.LEFT)

    camera_button_10189 = Tkinter.Button(frame, text="Camera Prime 17W #10189", command=lambda: choose_camera('10189'))
    camera_button_10189.pack(side=Tkinter.LEFT)

    camera_button_10910 = Tkinter.Button(frame, text="Camera Prime 13W #10910", command=lambda: choose_camera('10910'))
    camera_button_10910.pack(side=Tkinter.LEFT)

    print "hello world"

    Tkinter.mainloop()

    return cam

    ##root.quit() #doesnt work, the mainloop does not quit that way


def get_video_writer(cam, video_file='video.avi'):
    """Show video of camera's frame buffer"""

    #Define the codec and create VideoWriter object
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
                             video_file,           # Filename
                             CODEC,                # Codec for compression
                             cam.frame_rate,       # Frames per second
                             cam.frame_resolution, # Width / Height tuple
                             False                 # Color flag
                             )
    return writer

def write_video(cam, writer, record_time, save_video=True):
    """
    If save_video=True, which is default, function shows video and writes it to file until
    end of record_time or Escape key or q is pressed.
    If save_video=False function only shows video and independent of record_time.
    """
    start_time=time.time()
    while(True):
        k=cv2.waitKey(1)
        m.update()
        frame=cam.get_frame_buffer()
        cv2.imshow('Live Video. Framerate={}Hz'.format(cam.frame_rate),frame)
        #frame=np.random.rand(480,640)
        #cv2.imshow('frame',frame)

        if save_video:
            writer.write(frame)
            if time.time()>start_time+record_time:
                break

        if k in {27, ord('q')}:  #Hit Escape Key (depending on OS, escape code might not be 27) or q to exit
            break

    if save_video:
        print "Wrote video to file"
    else:
        print "Did not write video to file"


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="This is the motive camera video script. It can show and save the video data from any camera.",
                                     epilog="If no arguments are given, the script first opens a window to let you search for a project file to load. \n")

    parser.add_argument('-p', action='store', dest='project_file', default='',
                        help='Name of the project file to load.')

    parser.add_argument('-c', action='store', dest='camera_name', default='',
                        help='Name of the camera from which to get the video.')

    parser.add_argument('-f', action='store', dest='video_filename', default='video.avi',
                        help='Name of the file the video will be saved as.')

    parser.add_argument('-t', action='store', dest='record_time', default=60,
                        help='Maximum recording time (equals actual recording time if not stopped manually).')

    parser.add_argument('-s', action='store_false', dest='save_video', default=True,
                        help='If this flag is set, the video is shown but not saved.')

    args = parser.parse_args()


    m.load_project(args.project_file) if args.project_file else m.load_project(gui_load_project_file())

    if args.camera_name:
        cam=get_cam(args.camera_name)
    else:
        cam=gui_camera_name()

    #Get Video Writer
    if args.save_video:
        writer=get_video_writer(cam, args.video_filename)

    write_video(cam, writer, args.record_time, args.save_video) #again giving cam as argument, since writer object
                                                                #has no property to extract frame rate and frame_size from
    if args.save_video:
        writer.release()
    cv2.destroyAllWindows()
    m.shutdown()