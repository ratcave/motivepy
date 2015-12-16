import cv
import cv2
import time
import numpy as np
import argparse


def get_video_writer(video_file='video.avi', frame_rate=30, frame_resolution=(640,480)):
    """Show video of camera's frame buffer"""

    #Define the codec and create VideoWriter object
    #CODEC = cv.CV_FOURCC('D','I','V','3') # MPEG 4.3
    #CODEC = cv.CV_FOURCC('M','P','4','2') # MPEG 4.2
    #CODEC = cv.CV_FOURCC('M','J','P','G') # Motion Jpeg
    #CODEC = cv.CV_FOURCC('U','2','6','3') # H263
    #CODEC = cv.CV_FOURCC('I','2','6','3') # H263I
    #CODEC = cv.CV_FOURCC('F','L','V','1') # FLV
    #CODEC = cv.CV_FOURCC('P','I','M','1') # MPEG-1
    #CODEC = cv.CV_FOURCC('D','I','V','X') # MPEG-4 = MPEG-1
    CODEC=-1

    # Initialize the video writer to write the file
    writer = cv2.VideoWriter(
                             video_file,          # Filename
                             CODEC,               # Codec for compression
                             frame_rate,          # Frames per second
                             frame_resolution,    # Width / Height tuple
                             False                # Color flag
                             )
    return writer

def write_video(writer, record_time=10, save_video=True):
    start_time=time.time()
    while(True):
        k=cv2.waitKey(1)
        #native.update()
        #frame=self.get_frame_buffer()
        #cv2.imshow('framerate={}'.format(self.frame_rate),self.get_frame_buffer())
        frame=np.random.rand(480,640)
        cv2.imshow('frame',frame)

        if save_video:
            writer.write(frame)
            if time.time()>start_time+record_time:
                break

        if k in {27, ord('q')}:  #Hit Escape Key (value might not be 27 depending on OS) or q to exit
            break

    if save_video:
        print "Wrote Video To File"
    else:
        print "Did Not Write Video To File"

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="This is the motive camera video script. It can show and save the video data from any camera.",
                                     epilog="If no arguments are given, the script first opens a window to let you search for a project file to load. \n")

    #parser.camera to get video from


    parser.add_argument('-f', action='store', dest='video_file', default='video.avi',
                        help='Name of the file the video will be saved as.')

    parser.add_argument('-t', action='store', dest='record_time', default=60,
                        help='Maximum recording time (equals actual recording time if not stopped manually).')

    parser.add_argument('-s', action='store_false', dest='save_video', default=True,
                        help='If this flag is set, the video is shown but not saved.')

    args = parser.parse_args()

    #Get Video Writer
    if args.save_video:
        writer=get_video_writer(args.video_file)

    write_video(writer, args.record_time, args.save_video)
    if args.save_video:
        writer.release()
    cv2.destroyAllWindows()