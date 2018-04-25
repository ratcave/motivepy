"""Motive C3D Writer Module

This module features functionality to
create new c3d files from scratch and fill
them with marker position data.

Example::

    >>>acq=create_record_object()
    >>>update()
    >>>updated_acq=record_frames_markers(acq)
    >>>save_record_c3d(updated_acq, "test.c3d")

Note:
    Call update() (see native.pyx for more information on this function) to get
    the newest marker data.
"""
from __future__ import absoluate_import

import btk
from ..native import get_frame_markers

def create_record_object():
    """Returns a data aquisition object consisting of the marker positions of one frame"""
    acq=btk.btkAcquisition()
    btk.btkAcquisition.Reset(acq)
    btk.btkAcquisition.Init(acq,len(get_frame_markers()),1)

    return acq

def record_frames_markers(acq):
    """Writes the marker positions of one frame to an existing acquisition object
    and returns the updated acquisition object.

    Args:
        acq: Data acquisition object

    Returns:
        Data acquisition object
    """
    frames=acq.GetPointFrameNumber()
    markers=get_frame_markers()
    record_points=acq.GetPoints()

    if record_points.GetItemNumber()<len(markers):
        btk.btkAcquisition.ResizePointNumber(acq, len(markers))

    for i in xrange(len(markers)):
        point=acq.GetPoint(i)
        point.SetDataSlice(frames-1, *[1000*x for x in markers[i]])
    btk.btkAcquisition.ResizeFrameNumber(acq, frames+1)   #always one frame more than needed

    return acq

#TODO: def is_recording():

def save_record_c3d(acq, file_name):
    """Writes the data of an acquisition object to a c3d file.

    Args:
        acq: Data acquisition object
        file_name(str): Name of the c3d file

    Note:
        The file name extension should be .c3d
    """
    writer = btk.btkAcquisitionFileWriter()
    writer.SetInput(acq)
    writer.SetFilename(file_name)
    writer.Update()



