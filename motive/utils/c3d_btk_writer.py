__author__ = 'Nico'

import btk

def create_record_object():
    acq=btk.btkAcquisition()
    btk.btkAcquisition.Reset(acq)
    btk.btkAcquisition.Init(acq,len(m.get_frame_markers()),1)

    return acq

def record_frames_markers(acq):
    frames=acq.GetPointFrameNumber()
    markers=m.get_frame_markers()
    record_points=acq.GetPoints()

    if record_points.GetItemNumber()<len(markers):
        btk.btkAcquisition.ResizePointNumber(acq, len(markers))

    for i in xrange(len(markers)):
        point=acq.GetPoint(i)
        point.SetDataSlice(frames-1, markers[i])
    btk.btkAcquisition.ResizeFrameNumber(acq, frames+1)

    return acq

# def is_recording():

def save_record_c3d(acq, file_name):
    writer = btk.btkAcquisitionFileWriter()
    writer.SetInput(acq)
    writer.SetFilename(file_name)
    writer.Update()


