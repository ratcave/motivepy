__author__ = 'Nico'

import btk

def start_recording():
    acq=btk.btkAcquisition()
    btk.btkAcquisition.Reset(acq)
    btk.btkAcquisition.Init(acq,len(m.get_frame_markers()),1)

def record_frames_markers():

    for i in len(m.get_frame_markers()):
        point=acq.GetPoint(i)
        frames=acq.GetPointFrameNumber()
        x,y,z=m.get_frame_markers[i]
        point.SetValue(frames,0,x)
        point.SetValue(frames,1,y)
        point.SetValue(frames,2,z)




