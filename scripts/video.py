import cv
import cv2
import time
import numpy as np
import argparse
import motive as m


def get_cam(camera_name="91"):
    for cam in m.get_cams():
        if camera_name in cam.name:
            return cam

def get_video_writer(video_file='video.avi'):
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

def write_video(writer, record_time, save_video=True):
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
        cv2.imshow('framerate={}'.format(cam.frame_rate),frame)
        #frame=np.random.rand(480,640)
        #cv2.imshow('frame',frame)

        if save_video:
            writer.write(frame)
            if time.time()>start_time+record_time:
                break

        if k in {27, ord('q')}:  #Hit Escape Key (value might not be 27 depending on OS) or q to exit
            break

    if save_video:
        print "Wrote video to file"
    else:
        print "Did not write video to file"


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="This is the motive camera video script. It can show and save the video data from any camera.",
                                     epilog="If no arguments are given, the script first opens a window to let you search for a project file to load. \n")

    parser.add_argument('-p', action='store', dest='project_file', default=m.utils.backup_project_filename,
                        help='Name of the project file to load.')

    parser.add_argument('-c', action='store', dest='camera_name', default='91',
                        help='Name of the camera from which to get the video.')

    parser.add_argument('-f', action='store', dest='video_filename', default='video.avi',
                        help='Name of the file the video will be saved as.')

    parser.add_argument('-t', action='store', dest='record_time', default=60,
                        help='Maximum recording time (equals actual recording time if not stopped manually).')

    parser.add_argument('-s', action='store_false', dest='save_video', default=True,
                        help='If this flag is set, the video is shown but not saved.')

    args = parser.parse_args()


    m.load_project(args.project_file)
    cam=get_cam(args.camera_name)

    #Get Video Writer
    if args.save_video:
        writer=get_video_writer(args.video_filename)

    write_video(writer, args.record_time, args.save_video)
    if args.save_video:
        writer.release()
    cv2.destroyAllWindows()
    m.shutdown()