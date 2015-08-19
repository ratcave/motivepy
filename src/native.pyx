#include "cnative.pxd"

from libcpp cimport bool

cdef extern from "NPTrackingTools.h":

   cpdef int   TT_FrameMarkerCount()

   cpdef double TT_FrameTimeStamp()