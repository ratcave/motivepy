


from libcpp cimport bool


cdef extern from "NPTrackingTools.h":

    cpdef int    TT_FrameMarkerCount()

    cpdef double TT_FrameTimeStamp()

    cpdef bool   TT_CreateCameraGroup()  #Add an additional camera group

    cpdef int    TT_Initialize()

    cpdef int    TT_Shutdown()