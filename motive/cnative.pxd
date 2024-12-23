"""Motive Base Module

This module declares the basic functions and classes available
from the Motive C++ API. Once declared we can use them in specific
python modules.

Example:
    This example shows how to define a python function which is callable
    after the specific module is imported.

        >>> def update():
                return Update()

That function calls the C++ function declared in this module
(for more information see the documentation of native.pyx).
"""

from libcpp cimport bool
from libc.stddef cimport wchar_t

cdef extern from "MotiveAPI.h" namespace "Core":
    cdef cppclass cUID:
        unsigned long long int LowBits()
        unsigned long long int HighBits()


cdef extern from "MotiveAPI.h" namespace "MotiveAPIProcessingSettings":  #can not define enum in class in cython yet
          cdef enum Setting:            #unsigned long long
            eResolvePointCloud,         #bool
            eShowCameras,               #bool
            eVisibleMarkerSize,         #double Not to be found in GUI
            ePCResidual,                #double in meters
            ePCMinSize,                 #double same as below
            ePCMaxSize,                 #double is probably max marker size, but shows value different from GUI
            ePCMinAngle,                #double
            ePCMinRays,                 #long
            eShutterDelay,              #long shutter offset in micros (compare to shutter delay function!)
            ePrecisionPacketCap,        #long Not to be found in GUI
            ePCMinRayLength,            #double
            ePCMaxRayLength,            #double
            ePCReconstructMinX,         #double
            ePCReconstructMaxX,         #double
            ePCReconstructMinY,         #double
            ePCReconstructMaxY,         #double
            ePCReconstructMinZ,         #double
            ePCReconstructMaxZ,         #double
            ePCObjectFilterLevel,       #long 0=None, 2=Size & Roundness
            ePCObjectFilterMinSize,     #long
            ePCObjectFilterMaxSize,     #long
            ePCObjectFilterCircularity, #double
            ePCObjectFilterGrayscaleFloor, #long Not to be found in GUI
            ePCObjectFilterAspectTolerance, #long Not to be found in GUI
            ePCObjectFilterObjectMargin, #long Not to be found in GUI
            eShowReconstructionBounds,  #bool
            eBoundReconstruction,       #bool
            eShowCaptureVolume,         #bool
            eShow3DMarkers,             #bool
            eShowCameraFOV,             #bool
            eCameraOverlap,             #double This seems to be wrong type?! for setter (the wrong type guys seems to have actually three or four values to chooose from)
            eVolumeResolution,          #double This seems to be wrong type?! for setter
            eWireframe,                 #double opacity in pointcloudgroup
            eFOVIntensity,              #double
            eRankRays,                  #bool Not to be found in GUI as boolean. and when changed in API does not change in GUI
            eMinimumRankRayCount,       #long Not to be found in GUI
            ePCPixelGutter,             #long
            ePCMaximum2DPoints,         #long
            ePCCalculationTime,         #long
            ePCThreadCount,             #long
            ePCCalculateDiameter,       #bool
            ePCBoostMultThreads,        #bool
            ePCSmallMarkerOptimization, #long  0=None, 1=Fast, 2=Accurate
            eBlockWidth,                #double
            eBlockHeight,               #double
            ePointCloudEngine,          #long 1=v1.0  2=v2.0
            eSynchronizerEngine,        #long 1=v1.0  2=v2.0 This seems to be wrong type?! Neither getter nor setter works
            eMarkerDiameterType,        #long  Might be marker filter diameter, but shows value different from GUI
            eMarkerDiameterForceSize,   #double Might be min diameter, but shows value different from GUI
            eSynchronizerControl,       #long  0=timely delivery, 1=automatic, 2=complete delivery
            ePCBoostLeastSq,            #bool   This seems to be wrong type?! Neither getter nor setter works
            eSettingsCount              # returns 1


cdef extern from "MotiveAPI.h" namespace "MotiveAPI":

    cdef cppclass MotiveAPIProcessingSettings:
        MotiveAPIProcessingSettings() except +

        #Set individual parameter values. Only values that are set will be changed when submitting
        #the settings block to SetCameraGroupPointCloudSettings. These methods will return false
        #if there is a mismatch between the requested parameter and its expected type
        bool            SetBoolParameter( Setting which, bool value)
        bool            SetDoubleParameter( Setting which , double value )
        bool            SetLongParameter( Setting which, long value )

        #Retrieve individual parameter settings from the parameter block. These methods will return false
        #if there is a mismatch between the requested parameter and its expected type.
        bool            BoolParameter( Setting which, bool & value ) const
        bool            DoubleParameter( Setting which , double & value) const
        bool            LongParameter( Setting which , long & value) const

    cdef enum eResult:
        kApiResult_Success = 0,
        kApiResult_Failed,
        kApiResult_FileNotFound,
        kApiResult_LoadFailed,
        kApiResult_SaveFailed,
        kApiResult_InvalidFile,
        kApiResult_InvalidLicense,
        kApiResult_NoFrameAvailable,
        kApiResult_TooFewMarkers,
        kApiResult_CouldNotFindGroundPlane,
        kApiResult_UnableToAccessCameras

    cdef enum eVideoType:
        kVideoType_Segment   = 0,
        kVideoType_Grayscale = 1,
        kVideoType_Object    = 2,
        kVideoType_Precision = 4,
        kVideoType_MJPEG     = 6,
        kVideoType_ColorH264 = 9

    cdef enum eCameraState:
        Camera_Enabled = 0,
        Camera_Disabled_For_Reconstruction = 1,
        Camera_Disabled = 2

#STARTUP / SHUTDOWN
    eResult    Initialize()                                                        #initialize library
    void       Shutdown()                                                          #shutdown library

#RIGID BODY INTERFACE
    eResult    LoadCalibration(const wchar_t *filename, int *cameraCount)                                #load calibration
    ##int    LoadCalibrationW(const wchar_t *filename)                          ##only necessary when not using english alphabet to name files
    eResult    LoadRigidBodies(const wchar_t *filename)                                #load rigid bodies
    ##int    LoadRigidBodiesW(const wchar_t *filename)
    eResult    SaveRigidBodies(const char *filename)                                #save rigid bodies
    ##int    SaveRigidBodiesW(const wchar_t *filename)
    eResult    AddRigidBodies(const char *filename)                                 #add rigid bodies
    ##int    AddRigidBodiesW (const wchar_t *filename)
    eResult    LoadProfile(const wchar_t *filename)                                    #load profile file
    ##int    LoadProfileW(const wchar_t *filename)
    eResult    SaveProfile(const wchar_t *filename)                                    #save profile file
    ##int    SaveProfileW(const wchar_t *filename)
    eResult    LoadCalibrationFromMemory(unsigned char* buffer, int bufferSize)
    eResult    Update()                                                             # Process incoming camera data
    eResult    UpdateSingleFrame()                                                  # Process incoming camera data

#DATA STREAMING
#    int    StreamTrackd(bool enabled)                                           #Start/stop Trackd Stream
#    int    StreamVRPN(bool enabled, int port)                                   #Start/stop VRPN Stream
#    int    StreamNP(bool enabled)                                               #Start/stop NaturalPoint Stream

#FRAME
    int    MarkerCount()                                                   #Returns Frame Markers Count
#    float  FrameMarkerX(int index)                                              #Returns X Coord of Marker
#    float  FrameMarkerY(int index)                                              #Returns Y Coord of Marker
#    float  FrameMarkerZ(int index)                                              #Returns Z Coord of Marker
#    cUID FrameMarkerLabel(int index)                                          #Returns Label of Marker
    bool    MarkerXYZ( int markerIndex, float& x, float& y, float& z)
    double FrameTimeStamp()                                                     #Time Stamp of Frame (seconds)
    bool   FrameCameraCentroid(int index, int cameraIndex, float &x, float &y)  #FrameCameraCentroid returns true if the camera is contributing to this 3D marker.  It also returns the location of the 2D centroid that is reconstructing to this 3D marker ##through changing the x and y values
    void   FlushCameraQueues()                                                  #In the event that you are tracking a very high number of 2D and/or 3D markers (hundreds of 3D markers), and you find that the data you're getting out has sufficient latency you can call FlushCameraQueues() to catch up before calling Update(). Ideally, after calling FlushCameraQueues() you'll want to not call it again until after Update() returns NPRESULT_SUCCESS

#RIGID BODY CONTROL
    bool   IsRigidBodyTracked(int index)                                        #Is rigid body currently tracked
    void   RigidBodyTransform(int index,                                         #RigidBody Index
                                     float *x, float *y, float *z,                      #Position
                                     float *qx, float *qy, float *qz, float *qw,        #Orientation
                                     float *yaw, float *pitch, float *roll)             #Orientation

    eResult    RemoveRigidBody(int index)                                           #Remove single rigid body
    int    RigidBodyCount()                                                     #Returns number of rigid bodies
#    int    RigidBodyUserData(int index)                                         #Get RigidBodies User Data
#    void   SetRigidBodyUserData(int index, int ID)                              #Set RigidBodies User Data
    bool   RigidBodyName( int index, wchar_t* buffer, int bufferSize );
    #const char*  RigidBodyName (int index)                                      #Returns RigidBody Name
    ##const wchar_t* RigidBodyNameW(int index)
    void   SetRigidBodyEnabled(int index, bool enabled)                         #Set Tracking
    bool   RigidBodyEnabled(int index)                                          #Get Tracking
    eResult    RigidBodyTranslatePivot(int index, float x, float y, float z)
    bool   RigidBodyResetOrientation(int index)
    int    RigidBodyMarkerCount(int index)                                      #Get marker count
    void   RigidBodyMarker(int rigidIndex,                                      #Get RigidBody mrkr
                              int markerIndex, float *x, float *y, float *z)
#    void   RigidBodyPointCloudMarker(int rigidIndex,                            #Get corresponding point cloud marker
#                                             int markerIndex, bool &tracked,            #If tracked is false, there is no
#                                             float &x, float &y, float &z)              #corresponding point cloud marker.
    int    CreateRigidBody(const wchar_t* name, int id,                            #Create a rigid body based on the marker count and marker list provided.  The marker list is expected to contain of list of marker coordinates in the order: x1,y1,z1,x2,y2,z2,...xN,yN,zN.
                                   int markerCount, float *markerList)
    ##cdef int RigidBodySettings   (int index, cRigidBodySettings &settings)           #Get RigidBody Settings
    ##cdef int SetRigidBodySettings(int index, cRigidBodySettings &settings)           #Set RigidBody Settings
    ##CameraLibrary::CameraManager* GetCameraManager()                                 #Returns a pointer to the Camera SDK's CameraManager
    int	     BuildNumber()                                                       #Software Release Build

#CAMERA GROUP SUPPORT
    int    CameraGroupCount()                                                    #Returns number of camera groups
    int    CameraGroup(int index)                                               #Returns Camera's camera group index

#CAMERA GROUP FILTER SETTINGS
    ##int  CameraGroupFilterSettings(int groupIndex, cCameraGroupFilterSettings &settings)
    ##int  SetCameraGroupFilterSettings(int groupIndex, cCameraGroupFilterSettings &settings)

#POINT CLOUD RECONSTRUCTION SETTINGS
#    int CameraGroupPointCloudSettings   (int groupIndex, cCameraGroupPointCloudSettings &settings)
#    int SetCameraGroupPointCloudSettings(int groupIndex, cCameraGroupPointCloudSettings &settings)

#MARKER SIZE SETTINGS
    ##int CameraMarkerSize   (int groupIndex, MotiveAPIMarkerSizeSettings &settings)
    ##int SetCameraMarkerSize(int groupIndex, MotiveAPIMarkerSizeSettings &settings)
    int  SetCameraGroupReconstruction(int groupIndex, bool enable)
    int  SetEnabledFilterSwitch(bool enabled)
    bool IsFilterSwitchEnabled()

#POINT CLOUD INTERFACE

    bool SetCameraState( int cameraIndex, eCameraState state )
    bool CameraState( int cameraIndex, eCameraState& currentState )

    int    CameraCount()                                                         #Returns Camera Count
    float  CameraXLocation(int index)                                            #Returns Camera's X Coord
    float  CameraYLocation(int index)                                            #Returns Camera's Y Coord
    float  CameraZLocation(int index)                                            #Returns Camera's Z Coord
    float  CameraOrientationMatrix(int camera, int index)                        #Orientation
    bool   CameraName(int cameraIndex, wchar_t* buffer, int bufferSize )         #Returns Camera Name
    int    CameraMarkerCount(int cameraIndex)                                    #Camera's 2D Marker Count
    bool   CameraMarker(int cameraIndex, int markerIndex, float &x, float &y)    #CameraMarker fetches the 2D centroid location of the marker as seen by the camera.
    bool   CameraPixelResolution(int cameraIndex, int &width, int &height)
    bool   SetCameraSettings(int cameraIndex, int videoType, int exposure,       #VideoType: 0 = Segment Mode, 1 = Grayscale Mode, 2 = Object Mode, 4 = Precision Mode, 6 = MJPEG Mode. Exposure: Valid values are:  1-480. Threshold: Valid values are: 0-255. Intensity: Valid values are: 0-15  (This should be set to 15 for most situations)
                                int threshold, int intensity)
    bool   SetCameraSystemFrameRate(int frameRate)                    #Set the frame rate for the given zero based camera index. Returns true if the operation was successful and false otherwise. If the operation fails check that the camera index is valid and that devices have been initialized with Initialize()
    int    CameraVideoType(int cameraIndex)
    int    CameraSystemFrameRate()                                               #frames/sec
    int    CameraExposure(int cameraIndex)
    int    CameraThreshold(int cameraIndex)
    float  CameraTemperature(int cameraIndex)
    float  CameraRinglightTemperature(int cameraIndex)
    int    CameraGrayscaleDecimation(int cameraIndex)
    bool   SetCameraGrayscaleDecimation(int cameraIndex, int value)
    bool   SetCameraFilterSwitch(int cameraIndex, bool enableIRFilter)
    bool   SetCameraAGC(int cameraIndex, bool enableAutomaticGainControl)
    bool   SetCameraAEC(int cameraIndex, bool enableAutomaticExposureControl)
    bool   SetCameraHighPower(int cameraIndex, bool enableHighPowerMode)
    bool   SetCameraMJPEGQuality(int cameraIndex, int mjpegQuality)
    int    CameraImagerGain(int cameraIndex)
    int    CameraImagerGainLevels(int cameraIndex)
    void   SetCameraImagerGain(int cameraIndex, int value)
    bool   SetCameraIRLedsOn( int cameraIndex, bool irLedsOn )
    bool   CameraIRLedsOn( int cameraIndex )
    bool   ClearCameraMask(int cameraIndex)
    bool   SetCameraMask(int cameraIndex, unsigned char * buffer, int bufferSize)
    bool   CameraMask(int cameraIndex, unsigned char * buffer, int bufferSize)
    bool   CameraMaskInfo(int cameraIndex, int &blockingMaskWidth, int &blockingMaskHeight, int &blockingMaskGrid)
    int    CameraID(int cameraIndex)
    bool   CameraFrameBuffer(int cameraIndex, int bufferPixelWidth, int bufferPixelHeight,      #Fetch the camera's frame buffer.  This function fills the provided buffer with an image of what is in the camera view. The resulting image depends on what video mode the camera is in.  If the camera is in grayscale mode, for example, a grayscale image is returned from this call.
                                int bufferByteSpan, int bufferPixelBitDepth, unsigned char *buffer)
    void   CameraBackproject(int cameraIndex, float x, float y, float z,                        #Back-project from 3D space to 2D space.  If you give this function a 3D location and select a camera, it will return where the point would land on the imager of that camera in to 2D space. This basically locates where in the camera's FOV a 3D point would be located.
                                float &cameraX, float &cameraY)
    void   CameraUndistort2DPoint(int cameraIndex, float &x, float &y)
    void   CameraDistort2DPoint  (int cameraIndex, float &x, float &y)
    bool   CameraRay(int cameraIndex, float x, float y,
                        float &rayStartX, float &rayStartY, float &rayStartZ,
                        float &rayEndX,   float &rayEndY,   float &rayEndZ)

    #Set a camera's extrinsic (position & orientation) and intrinsic (lens distortion) parameters with parameters compatible with the OpenCV intrinsic model.
    bool   CameraModel(int cameraIndex, float x, float y, float z,              #Camera Position
                          float *orientation,                                      #Orientation (3x3 matrix)
                          float principleX, float principleY,                      #Lens center (in pixels)
                          float focalLengthX, float focalLengthY,                  #Lens focal  (in pixels)
                          float kc1, float kc2, float kc3,                         #Barrel distortion coefficients
                          float tangential0, float tangential1)                    #Tangential distortion

    #This function will return the Camera SDK's camera pointer.  While the API takes over the data path which prohibits fetching the frames directly from the camera, it is still very useful to be able to communicate with the camera directly for setting camera settings or attaching modules.
    ##CameraLibrary::Camera * GetCamera(int index)                                #Returns Camera SDK Camera

    bool   SetFrameIDBasedTiming(bool enable)
    bool   SetSuppressOutOfOrder(bool enable)
    ##void   AttachCameraModule(int index, CameraLibrary::cCameraModule *module)
    ##void   DetachCameraModule(int index, CameraLibrary::cCameraModule *module)
    int    OrientTrackingBar(float positionX, float positionY, float positionZ,
                                float orientationX, float orientationY, float orientationZ, float orientationW)
    ##void     AttachRigidBodySolutionTest(int index, cRigidBodySolutionTest* test)
    ##void     DetachRigidBodySolutionTest(int index, cRigidBodySolutionTest* test)
    ##void     AttachListener(cTTAPIListener* listener)
    ##void     DetachListener(cTTAPIListener* listener)


#    int NPVIDEOTYPE_SEGMENT
#    int NPVIDEOTYPE_GRAYSCALE
#    int NPVIDEOTYPE_OBJECT
#    int NPVIDEOTYPE_PRECISION
#    int NPVIDEOTYPE_MJPEG

#    int NPRESULT_SUCCESS
#    int NPRESULT_FILENOTFOUND
#    int NPRESULT_LOADFAILED
#    int NPRESULT_FAILED
#    int NPRESULT_INVALIDFILE
#    int NPRESULT_INVALIDCALFILE
#    int NPRESULT_UNABLETOINITIALIZE
#    int NPRESULT_INVALIDLICENSE
#    int NPRESULT_NOFRAMEAVAILABLE