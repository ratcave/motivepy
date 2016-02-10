"""Motive Base Module

This module declares the basic functions and classes available
from the Motive C++ API. Once declared we can use them in specific
python modules.

Example:
    This example shows how to define a python function which is callable
    after the specific module is imported.

        >>> def update():
                return TT_Update()

That function calls the C++ function declared in this module
(for more information see the documentation of native.pyx).
"""

from libcpp cimport bool


cdef extern from "NPTrackingTools.h" namespace "Core":
    cdef cppclass cUID:
        unsigned long long int LowBits()
        unsigned long long int HighBits()

cdef extern from "NPTrackingTools.h" namespace "cCameraGroupPointCloudSettings":  #can not define enum in class in cython yet
          #unsigned long long k=1 #can not use that as 1LL since in enum I need constant expression
          cdef enum Setting:
#         #unsigned long long
            eResolvePointCloud,       #= 100000000000000000000000000000000L# L!!!              #bool
            eShowCameras,             #= 100000000000000000000000000000000L << 1               #bool
            eVisibleMarkerSize,          #double
            ePCResidual,                 #double
            ePCMinSize,                  #double
            ePCMaxSize,                  #double
            ePCMinAngle,                 #double
            ePCMinRays,                  #long
            eShutterDelay,               #long
            ePrecisionPacketCap,        #long
            ePCMinRayLength,            #double
            ePCMaxRayLength,            #double
            ePCReconstructMinX,         #double
            ePCReconstructMaxX,         #double
            ePCReconstructMinY,         #double
            ePCReconstructMaxY,         #double
            ePCReconstructMinZ,         #double
            ePCReconstructMaxZ,         #double
            ePCObjectFilterLevel,       #long
            ePCObjectFilterMinSize,     #long
            ePCObjectFilterMaxSize,     #long
            ePCObjectFilterCircularity, #double
            ePCObjectFilterGrayscaleFloor, #long
            ePCObjectFilterAspectTolerance, #long
            ePCObjectFilterObjectMargin, #long
            eShowReconstructionBounds,  #bool
            eBoundReconstruction,       #bool
            eShowCaptureVolume,         #bool
            eShow3DMarkers,             #bool
            eShowCameraFOV,             #bool
            eCameraOverlap,             #double
            eVolumeResolution,          #double
            eWireframe,                 #double
            eFOVIntensity,              #double
            eRankRays,                  #bool
            eMinimumRankRayCount,       #long
            ePCPixelGutter,             #long
            ePCMaximum2DPoints,         #long
            ePCCalculationTime,         #long
            ePCThreadCount,             #long
            ePCCalculateDiameter,       #bool
            ePCBoostMultThreads,        #bool
            ePCSmallMarkerOptimization, #long
            eBlockWidth,                #double
            eBlockHeight,               #double
            ePointCloudEngine,          #long 1=v1.0  2=v2.0
            eSynchronizerEngine,        #long 1=v1.0  2=v2.0
            eMarkerDiameterType,        #long
            eMarkerDiameterForceSize,   #double
            eSynchronizerControl,       #long
            ePCBoostLeastSq,            #bool
            eSettingsCount


cdef extern from "NPTrackingTools.h":

    cdef cppclass cCameraGroupPointCloudSettings:
        cCameraGroupPointCloudSettings() except +

        #Setting settings=  eResolvePointCloud #without extra extern from above, Setting is not defined. Also  Cannot assign default value to fields in cdef classes, structs or unions


        #returns: wrong C syntax error
        #cdef enum Setting:
        #        eResolvePointCloud,       #= 100000000000000000000000000000000L# L!!!              #bool
        #        eShowCameras

        #Set individual parameter values. Only values that are set will be changed when submitting
        #the settings block to TT_SetCameraGroupPointCloudSettings. These methods will return false
        #if there is a mismatch between the requested parameter and its expected type
        bool            SetBoolParameter( Setting which, bool value)
        bool            SetDoubleParameter( setting , double )
        bool            SetLongParameter( setting, long )
        #
        #
        # #Retrieve individual parameter settings from the parameter block. These methods will return false
        # #if there is a mismatch between the requested parameter and its expected type.
        bool            BoolParameter( Setting which, bool & value ) const
        bool            DoubleParameter( setting , double & ) const
        bool            LongParameter( setting , long & ) const


#STARTUP / SHUTDOWN
    int    TT_Initialize()                                                        #initialize library
    int    TT_Shutdown()                                                          #shutdown library

#RIGID BODY INTERFACE
    int    TT_LoadCalibration(const char *filename)                                #load calibration
    ##int    TT_LoadCalibrationW(const wchar_t *filename)                          ##only necessary when not using english alphabet to name files
    int    TT_LoadRigidBodies(const char *filename)                                #load rigid bodies
    ##int    TT_LoadRigidBodiesW(const wchar_t *filename)
    int    TT_SaveRigidBodies(const char *filename)                                #save rigid bodies
    ##int    TT_SaveRigidBodiesW(const wchar_t *filename)
    int    TT_AddRigidBodies(const char *filename)                                 #add rigid bodies
    ##int    TT_AddRigidBodiesW (const wchar_t *filename)
    int    TT_LoadProject(const char *filename)                                    #load project file
    ##int    TT_LoadProjectW(const wchar_t *filename)
    int    TT_SaveProject(const char *filename)                                    #save project file
    ##int    TT_SaveProjectW(const wchar_t *filename)
    int    TT_LoadCalibrationFromMemory(unsigned char* buffer, int bufferSize)
    int    TT_Update()                                                             # Process incoming camera data
    int    TT_UpdateSingleFrame()                                                  # Process incoming camera data

#DATA STREAMING
    int    TT_StreamTrackd(bool enabled)                                           #Start/stop Trackd Stream
    int    TT_StreamVRPN(bool enabled, int port)                                   #Start/stop VRPN Stream
    int    TT_StreamNP(bool enabled)                                               #Start/stop NaturalPoint Stream

#FRAME
    int    TT_FrameMarkerCount()                                                   #Returns Frame Markers Count
    float  TT_FrameMarkerX(int index)                                              #Returns X Coord of Marker
    float  TT_FrameMarkerY(int index)                                              #Returns Y Coord of Marker
    float  TT_FrameMarkerZ(int index)                                              #Returns Z Coord of Marker
    cUID TT_FrameMarkerLabel(int index)                                          #Returns Label of Marker
    double TT_FrameTimeStamp()                                                     #Time Stamp of Frame (seconds)
    bool   TT_FrameCameraCentroid(int index, int cameraIndex, float &x, float &y)  #TT_FrameCameraCentroid returns true if the camera is contributing to this 3D marker.  It also returns the location of the 2D centroid that is reconstructing to this 3D marker ##through changing the x and y values
    void   TT_FlushCameraQueues()                                                  #In the event that you are tracking a very high number of 2D and/or 3D markers (hundreds of 3D markers), and you find that the data you're getting out has sufficient latency you can call TT_FlushCameraQueues() to catch up before calling TT_Update(). Ideally, after calling TT_FlushCameraQueues() you'll want to not call it again until after TT_Update() returns NPRESULT_SUCCESS

#RIGID BODY CONTROL
    bool   TT_IsRigidBodyTracked(int index)                                        #Is rigid body currently tracked
    void   TT_RigidBodyLocation(int index,                                         #RigidBody Index
                                     float *x, float *y, float *z,                      #Position
                                     float *qx, float *qy, float *qz, float *qw,        #Orientation
                                     float *yaw, float *pitch, float *roll)             #Orientation

    void   TT_ClearRigidBodyList()                                                 #Clear all rigid bodies
    int    TT_RemoveRigidBody(int index)                                           #Remove single rigid body
    int    TT_RigidBodyCount()                                                     #Returns number of rigid bodies
    int    TT_RigidBodyUserData(int index)                                         #Get RigidBodies User Data
    void   TT_SetRigidBodyUserData(int index, int ID)                              #Set RigidBodies User Data
    const char*  TT_RigidBodyName (int index)                                      #Returns RigidBody Name
    ##const wchar_t* TT_RigidBodyNameW(int index)
    void   TT_SetRigidBodyEnabled(int index, bool enabled)                         #Set Tracking
    bool   TT_RigidBodyEnabled(int index)                                          #Get Tracking
    int    TT_RigidBodyTranslatePivot(int index, float x, float y, float z)
    bool   TT_RigidBodyResetOrientation(int index)
    int    TT_RigidBodyMarkerCount(int index)                                      #Get marker count
    void   TT_RigidBodyMarker(int rigidIndex,                                      #Get RigidBody mrkr
                              int markerIndex, float *x, float *y, float *z)
    void   TT_RigidBodyPointCloudMarker(int rigidIndex,                            #Get corresponding point cloud marker
                                             int markerIndex, bool &tracked,            #If tracked is false, there is no
                                             float &x, float &y, float &z)              #corresponding point cloud marker.
    int    TT_CreateRigidBody(const char* name, int id,                            #Create a rigid body based on the marker count and marker list provided.  The marker list is expected to contain of list of marker coordinates in the order: x1,y1,z1,x2,y2,z2,...xN,yN,zN.
                                   int markerCount, float *markerList)
    ##cdef int TT_RigidBodySettings   (int index, cRigidBodySettings &settings)           #Get RigidBody Settings
    ##cdef int TT_SetRigidBodySettings(int index, cRigidBodySettings &settings)           #Set RigidBody Settings
    ##CameraLibrary::CameraManager* TT_GetCameraManager()                                 #Returns a pointer to the Camera SDK's CameraManager
    int	     TT_BuildNumber()                                                       #Software Release Build

#CAMERA GROUP SUPPORT
    int    TT_CameraGroupCount()                                                    #Returns number of camera groups
    bool   TT_CreateCameraGroup()                                                   #Add an additional group
    bool   TT_RemoveCameraGroup(int index)                                          #Remove a camera group (must be empty)
    int    TT_CamerasGroup(int index)                                               #Returns Camera's camera group index
    void   TT_SetGroupShutterDelay(int groupIndex, int microseconds)                #Set camera group's shutter delay
    void   TT_SetCameraGroup(int cameraIndex, int cameraGroupIndex)                 #Move camera to camera group
#CAMERA GROUP FILTER SETTINGS
    ##int  TT_CameraGroupFilterSettings(int groupIndex, cCameraGroupFilterSettings &settings)
    ##int  TT_SetCameraGroupFilterSettings(int groupIndex, cCameraGroupFilterSettings &settings)

#POINT CLOUD RECONSTRUCTION SETTINGS
    int TT_CameraGroupPointCloudSettings   (int groupIndex, cCameraGroupPointCloudSettings &settings)
    int TT_SetCameraGroupPointCloudSettings(int groupIndex, cCameraGroupPointCloudSettings &settings)

#MARKER SIZE SETTINGS
    ##int TT_CameraGroupMarkerSize   (int groupIndex, cCameraGroupMarkerSizeSettings &settings)
    ##int TT_SetCameraGroupMarkerSize(int groupIndex, cCameraGroupMarkerSizeSettings &settings)
    int  TT_SetCameraGroupReconstruction(int groupIndex, bool enable)
    int  TT_SetEnabledFilterSwitch(bool enabled)
    bool TT_IsFilterSwitchEnabled()

#POINT CLOUD INTERFACE
    int    TT_CameraCount()                                                         #Returns Camera Count
    float  TT_CameraXLocation(int index)                                            #Returns Camera's X Coord
    float  TT_CameraYLocation(int index)                                            #Returns Camera's Y Coord
    float  TT_CameraZLocation(int index)                                            #Returns Camera's Z Coord
    float  TT_CameraOrientationMatrix(int camera, int index)                        #Orientation
    const char* TT_CameraName(int index)                                            #Returns Camera Name
    int    TT_CameraMarkerCount(int cameraIndex)                                    #Camera's 2D Marker Count
    bool   TT_CameraMarker(int cameraIndex, int markerIndex, float &x, float &y)    #CameraMarker fetches the 2D centroid location of the marker as seen by the camera.
    bool   TT_CameraPixelResolution(int cameraIndex, int &width, int &height)
    bool   TT_SetCameraSettings(int cameraIndex, int videoType, int exposure,       #VideoType: 0 = Segment Mode, 1 = Grayscale Mode, 2 = Object Mode, 4 = Precision Mode, 6 = MJPEG Mode. Exposure: Valid values are:  1-480. Threshold: Valid values are: 0-255. Intensity: Valid values are: 0-15  (This should be set to 15 for most situations)
                                int threshold, int intensity)
    bool   TT_SetCameraFrameRate(int cameraIndex, int frameRate)                    #Set the frame rate for the given zero based camera index. Returns true if the operation was successful and false otherwise. If the operation fails check that the camera index is valid and that devices have been initialized with TT_Initialize()
    int    TT_CameraVideoType(int cameraIndex)
    int    TT_CameraFrameRate(int cameraIndex)                                      #frames/sec
    int    TT_CameraExposure(int cameraIndex)
    int    TT_CameraThreshold(int cameraIndex)
    int    TT_CameraIntensity(int cameraIndex)
    float  TT_CameraTemperature(int cameraIndex)
    float  TT_CameraRinglightTemperature(int cameraIndex)
    int    TT_CameraGrayscaleDecimation(int cameraIndex)
    bool   TT_SetCameraGrayscaleDecimation(int cameraIndex, int value)
    bool   TT_SetCameraFilterSwitch(int cameraIndex, bool enableIRFilter)
    bool   TT_SetCameraAGC(int cameraIndex, bool enableAutomaticGainControl)
    bool   TT_SetCameraAEC(int cameraIndex, bool enableAutomaticExposureControl)
    bool   TT_SetCameraHighPower(int cameraIndex, bool enableHighPowerMode)
    bool   TT_SetCameraMJPEGHighQuality(int cameraIndex, int mjpegQuality)
    int    TT_CameraImagerGain(int cameraIndex)
    int    TT_CameraImagerGainLevels(int cameraIndex)
    void   TT_SetCameraImagerGain(int cameraIndex, int value)
    bool   TT_IsContinuousIRAvailable(int cameraIndex)
    void   TT_SetContinuousTT_SetCameraMJPEGHighQualityIR(int cameraIndex, bool Enable)
    bool   TT_ContinuousIR(int cameraIndex)
    void   TT_SetContinuousIR(int cameraIndex, bool Enable)
    bool   TT_ClearCameraMask(int cameraIndex)
    bool   TT_SetCameraMask(int cameraIndex, unsigned char * buffer, int bufferSize)
    bool   TT_CameraMask(int cameraIndex, unsigned char * buffer, int bufferSize)
    bool   TT_CameraMaskInfo(int cameraIndex, int &blockingMaskWidth, int &blockingMaskHeight, int &blockingMaskGrid)
    int    TT_CameraID(int cameraIndex)
    bool   TT_CameraFrameBuffer(int cameraIndex, int bufferPixelWidth, int bufferPixelHeight,      #Fetch the camera's frame buffer.  This function fills the provided buffer with an image of what is in the camera view. The resulting image depends on what video mode the camera is in.  If the camera is in grayscale mode, for example, a grayscale image is returned from this call.
                                int bufferByteSpan, int bufferPixelBitDepth, unsigned char *buffer)
    bool   TT_CameraFrameBufferSaveAsBMP(int cameraIndex, const char *filename)                    #Save camera's frame buffer as a BMP image file
    void   TT_CameraBackproject(int cameraIndex, float x, float y, float z,                        #Back-project from 3D space to 2D space.  If you give this function a 3D location and select a camera, it will return where the point would land on the imager of that camera in to 2D space. This basically locates where in the camera's FOV a 3D point would be located.
                                float &cameraX, float &cameraY)
    void   TT_CameraUndistort2DPoint(int cameraIndex, float &x, float &y)
    void   TT_CameraDistort2DPoint  (int cameraIndex, float &x, float &y)
    bool   TT_CameraRay(int cameraIndex, float x, float y,
                        float &rayStartX, float &rayStartY, float &rayStartZ,
                        float &rayEndX,   float &rayEndY,   float &rayEndZ)
    #Set a camera's extrinsic (position & orientation) and intrinsic (lens distortion) parameters with parameters compatible with the OpenCV intrinsic model.
    bool   TT_CameraModel(int cameraIndex, float x, float y, float z,              #Camera Position
                          float *orientation,                                      #Orientation (3x3 matrix)
                          float principleX, float principleY,                      #Lens center (in pixels)
                          float focalLengthX, float focalLengthY,                  #Lens focal  (in pixels)
                          float kc1, float kc2, float kc3,                         #Barrel distortion coefficients
                          float tangential0, float tangential1)                    #Tangential distortion
    #This function will return the Camera SDK's camera pointer.  While the API takes over the data path which prohibits fetching the frames directly from the camera, it is still very useful to be able to communicate with the camera directly for setting camera settings or attaching modules.
    ##CameraLibrary::Camera * TT_GetCamera(int index)                                #Returns Camera SDK Camera
    bool   TT_SetFrameIDBasedTiming(bool enable)
    bool   TT_SetSuppressOutOfOrder(bool enable)
    ##void   TT_AttachCameraModule(int index, CameraLibrary::cCameraModule *module)
    ##void   TT_DetachCameraModule(int index, CameraLibrary::cCameraModule *module)
    int    TT_OrientTrackingBar(float positionX, float positionY, float positionZ,
                                float orientationX, float orientationY, float orientationZ, float orientationW)
    ##void     TT_AttachRigidBodySolutionTest(int index, cRigidBodySolutionTest* test)
    ##void     TT_DetachRigidBodySolutionTest(int index, cRigidBodySolutionTest* test)
    ##void     TT_AttachListener(cTTAPIListener* listener)
    ##void     TT_DetachListener(cTTAPIListener* listener)