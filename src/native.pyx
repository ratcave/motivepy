


from libcpp cimport bool


cdef extern from "NPTrackingTools.h":

    cpdef int    TT_Initialize()                              #initialize library
    cpdef int    TT_Shutdown()

    cpdef int    TT_LoadCalibration(const char *filename)

    cpdef int    TT_LoadRigidBodies(const char *filename)

    cpdef int    TT_SaveRigidBodies(const char *filename)

    cpdef int    TT_AddRigidBodies(const char *filename)


    cpdef int    TT_CameraCount()

    cpdef int    TT_CameraIntensity(int cameraIndex)

