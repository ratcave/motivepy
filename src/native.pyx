##one # means comment as in motive SDK, two # means comment by me

from libcpp cimport bool


cdef extern from "NPTrackingTools.h":


#STARTUP / SHUTDOWN
    cpdef int    TT_Initialize()                                                        #initialize library
    cpdef int    TT_Shutdown()                                                          #shutdown library

#RIGID BODY INTERFACE
    cdef int    TT_LoadCalibration(const char *filename)                                #load calibration
    ##cdef int    TT_LoadCalibrationW(const wchar_t *filename)                          ##only necessary when not using english alphabet to name files
    cdef int    TT_LoadRigidBodies(const char *filename)                                #load rigid bodies
    ##cdef int    TT_LoadRigidBodiesW(const wchar_t *filename)
    cdef int    TT_SaveRigidBodies(const char *filename)                                #save rigid bodies
    ##cdef int    TT_SaveRigidBodiesW(const wchar_t *filename)
    cdef int    TT_AddRigidBodies(const char *filename)                                 #add rigid bodies
    ##cdef int    TT_AddRigidBodiesW (const wchar_t *filename)
    cdef int    TT_LoadProject(const char *filename)                                    #load project file
    ##cdef int    TT_LoadProjectW(const wchar_t *filename)
    cdef int    TT_SaveProject(const char *filename)                                    #save project file
    ##cdef int    TT_SaveProjectW(const wchar_t *filename)
    cdef int    TT_LoadCalibrationFromMemory(unsigned char* buffer, int bufferSize)
    cdef int    TT_Update()                                                             # Process incoming camera data
    cdef int    TT_UpdateSingleFrame()                                                  # Process incoming camera data

#DATA STREAMING
    cdef int    TT_StreamTrackd(bool enabled)                                           #Start/stop Trackd Stream
    cdef int    TT_StreamVRPN(bool enabled, int port)                                   #Start/stop VRPN Stream
    cdef int    TT_StreamNP(bool enabled)                                               #Start/stop NaturalPoint Stream

#FRAME
    cdef int    TT_FrameMarkerCount()                                                   #Returns Frame Markers Count
    cdef float  TT_FrameMarkerX(int index)                                              #Returns X Coord of Marker
    cdef float  TT_FrameMarkerY(int index)                                              #Returns Y Coord of Marker
    cdef float  TT_FrameMarkerZ(int index)                                              #Returns Z Coord of Marker
    ##Core::cUID TT_FrameMarkerLabel(int index)                                          #Returns Label of Marker
    cdef double TT_FrameTimeStamp()                                                     #Time Stamp of Frame (seconds)
    ##here not sure about the C++ function argument &operator accessibility in python
    cdef bool   TT_FrameCameraCentroid(int index, int cameraIndex, float &x, float &y)  #TT_FrameCameraCentroid returns true if the camera is contributing to this 3D marker.  It also returns the location of the 2D centroid that is reconstructing to this 3D marker ##through changing the x and y values
    cdef void   TT_FlushCameraQueues()                                                  #In the event that you are tracking a very high number of 2D and/or 3D markers (hundreds of 3D markers), and you find that the data you're getting out has sufficient latency you can call TT_FlushCameraQueues() to catch up before calling TT_Update(). Ideally, after calling TT_FlushCameraQueues() you'll want to not call it again until after TT_Update() returns NPRESULT_SUCCESS

#RIGID BODY CONTROL
    cdef bool   TT_IsRigidBodyTracked(int index)                                        #Is rigid body currently tracked
    cdef void   TT_RigidBodyLocation(int index,                                         #RigidBody Index
                                     float *x, float *y, float *z,                      #Position
                                     float *qx, float *qy, float *qz, float *qw,        #Orientation
                                     float *yaw, float *pitch, float *roll)             #Orientation

    cdef void   TT_ClearRigidBodyList()                                                 #Clear all rigid bodies
    cdef int    TT_RemoveRigidBody(int index)                                           #Remove single rigid body
    cdef int    TT_RigidBodyCount()                                                     #Returns number of rigid bodies
    cdef int    TT_RigidBodyUserData(int index)                                         #Get RigidBodies User Data
    cdef void   TT_SetRigidBodyUserData(int index, int ID)                              #Set RigidBodies User Data
    cdef const char*  TT_RigidBodyName (int index)                                      #Returns RigidBody Name
    ##cdef const wchar_t* TT_RigidBodyNameW(int index)
    cdef void   TT_SetRigidBodyEnabled(int index, bool enabled)                         #Set Tracking
    cdef bool   TT_RigidBodyEnabled(int index)                                          #Get Tracking
    cdef int    TT_RigidBodyTranslatePivot(int index, float x, float y, float z)
    cdef bool   TT_RigidBodyResetOrientation(int index)
    cdef int    TT_RigidBodyMarkerCount(int index)                                      #Get marker count
    cdef void   TT_RigidBodyMarker(int rigidIndex,                                      #Get RigidBody mrkr
                                   int markerIndex, float *x, float *y, float *z)
    cdef void   TT_RigidBodyPointCloudMarker(int rigidIndex,                            #Get corresponding point cloud marker
                                             int markerIndex, bool &tracked,            #If tracked is false, there is no
                                             float &x, float &y, float &z)              #corresponding point cloud marker.
    cdef int    TT_CreateRigidBody(const char* name, int id,                            #Create a rigid body based on the marker count and marker list provided.  The marker list is expected to contain of list of marker coordinates in the order: x1,y1,z1,x2,y2,z2,...xN,yN,zN.
                                   int markerCount, float *markerList)
    ##cdef int TT_RigidBodySettings   (int index, cRigidBodySettings &settings)           #Get RigidBody Settings
    ##cdef int TT_SetRigidBodySettings(int index, cRigidBodySettings &settings)           #Set RigidBody Settings

#CAMERA GROUP SUPPORT
    cdef int    TT_CameraGroupCount()                                                     #Returns number of camera groups
    cdef bool   TT_CreateCameraGroup()                                                    #Add an additional group
    cdef bool   TT_RemoveCameraGroup(int index)                                           #Remove a camera group (must be empty)
    cdef int    TT_CamerasGroup(int index)                                                #Returns Camera's camera group index
    cdef void   TT_SetGroupShutterDelay(int groupIndex, int microseconds)                 #Set camera group's shutter delay
    cdef void   TT_SetCameraGroup(int cameraIndex, int cameraGroupIndex)                  #Move camera to camera group
#CAMERA GROUP FILTER SETTINGS
    ##cdef int  TT_CameraGroupFilterSettings(int groupIndex, cCameraGroupFilterSettings &settings)
    ##cdef int TT_SetCameraGroupFilterSettings(int groupIndex, cCameraGroupFilterSettings &settings)

#POINT CLOUD RECONSTRUCTION SETTINGS
    ##cdef int TT_CameraGroupPointCloudSettings   (int groupIndex, cCameraGroupPointCloudSettings &settings)
    ##cdef int TT_SetCameraGroupPointCloudSettings(int groupIndex, cCameraGroupPointCloudSettings &settings)

#MARKER SIZE SETTINGS
    ##cdef int TT_CameraGroupMarkerSize   (int groupIndex, cCameraGroupMarkerSizeSettings &settings)
    ##cdef int TT_SetCameraGroupMarkerSize(int groupIndex, cCameraGroupMarkerSizeSettings &settings)
    cdef int  TT_SetCameraGroupReconstruction(int groupIndex, bool enable)
    cdef int  TT_SetEnabledFilterSwitch(bool enabled)
    cdef bool TT_IsFilterSwitchEnabled()

#POINT CLOUD INTERFACE
    cdef int    TT_CameraCount()                                                         #Returns Camera Count
    cdef float  TT_CameraXLocation(int index)                                            #Returns Camera's X Coord
    cdef float  TT_CameraYLocation(int index)                                            #Returns Camera's Y Coord
    cdef float  TT_CameraZLocation(int index)                                            #Returns Camera's Z Coord
    cdef float  TT_CameraOrientationMatrix(int camera, int index)                        #Orientation
    cdef const char* TT_CameraName(int index)                                            #Returns Camera Name
    cdef int    TT_CameraMarkerCount(int cameraIndex)                                    #Camera's 2D Marker Count     ##cpdef bool   TT_CameraMarker(int cameraIndex, int markerIndex, float &x, float &y)    #CameraMarker fetches the 2D centroid location of the marker as seen by the camera.
    cdef bool     TT_CameraPixelResolution(int cameraIndex, int &width, int &height)
    cdef bool   TT_SetCameraSettings(int cameraIndex, int videoType, int exposure,       #VideoType: 0 = Segment Mode, 1 = Grayscale Mode, 2 = Object Mode, 4 = Precision Mode, 6 = MJPEG Mode. Exposure: Valid values are:  1-480. Threshold: Valid values are: 0-255. Intensity: Valid values are: 0-15  (This should be set to 15 for most situations)
                                       int threshold, int intensity)
    cdef bool   TT_SetCameraFrameRate(int cameraIndex, int frameRate)                    #Set the frame rate for the given zero based camera index. Returns true if the operation was successful and false otherwise. If the operation fails check that the camera index is valid and that devices have been initialized with TT_Initialize()
    cdef int    TT_CameraVideoType(int cameraIndex)
    cdef int    TT_CameraFrameRate(int cameraIndex)                                      # frames/sec
    cdef int    TT_CameraExposure(int cameraIndex)
    cdef int    TT_CameraThreshold(int cameraIndex)
    cdef int    TT_CameraIntensity(int cameraIndex)
    cdef float  TT_CameraTemperature(int cameraIndex)
    cdef float  TT_CameraRinglightTemperature(int cameraIndex)
    cdef int      TT_CameraGrayscaleDecimation(int cameraIndex)
    cdef bool     TT_SetCameraGrayscaleDecimation(int cameraIndex, int value)
    cdef bool     TT_SetCameraFilterSwitch(int cameraIndex, bool enableIRFilter)
    cdef bool     TT_SetCameraAGC(int cameraIndex, bool enableAutomaticGainControl)
    cdef bool     TT_SetCameraAEC(int cameraIndex, bool enableAutomaticExposureControl)

    cdef bool     TT_SetCameraHighPower(int cameraIndex, bool enableHighPowerMode)
    cdef bool     TT_SetCameraMJPEGHighQuality(int cameraIndex, int mjpegQuality)
    cdef int      TT_CameraImagerGain(int cameraIndex)
    cdef int      TT_CameraImagerGainLevels(int cameraIndex)
    cdef void     TT_SetCameraImagerGain(int cameraIndex, int value)
    cdef bool     TT_IsContinuousIRAvailable(int cameraIndex)
    cdef void     TT_SetContinuousTT_SetCameraMJPEGHighQualityIR(int cameraIndex, bool Enable)  ##linking error LNK
    cdef bool     TT_ContinuousIR(int cameraIndex)
    cdef void     TT_SetContinuousIR(int cameraIndex, bool Enable)

def set_camera_settings(self, camindex, videotype, exposure, threshold, intensity):
    return TT_SetCameraSettings(camindex, videotype, exposure, threshold, intensity)

def set_camera_group(self, camindex, camgroupindex):
    TT_SetCameraGroup(camindex, camgroupindex)
    print "set camera group"