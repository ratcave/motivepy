__author__ = 'Vash'

# means comment from official SDK API (or Cython page) ## means comment from me

include "cnative.pxd"


#STARTUP / SHUTDOWN
def initialize():
    """initialize library"""
    return TT_Initialize()

def shutdown():
    """shutdown library"""
    return TT_Shutdown()

#RIGID BODY INTERFACE ##FILES
def load_calibration(calib_filename):
    """load calibration"""
    assert type(calib_filename) is str, "Argument should be filename, i.e. a string literal"
    return TT_LoadCalibration(calib_filename)

def load_rigid_bodies(load_bodies_file):
    """load rigid bodies"""
    assert type(load_bodies_file) is str, "Argument should be filename, i.e. a string literal"
    return TT_LoadRigidBodies(load_bodies_file)

def save_rigid_bodies(rigid_filename):
    """save rigid bodies"""
    assert type(rigid_filename) is str, "Argument should be filename, i.e. a string literal"
    return TT_SaveRigidBodies(rigid_filename)

def add_rigid_bodies(rigid_bodies_file):
    """add rigid bodies"""
    assert type(rigid_bodies_file) is str, "Argument should be filename, i.e. a string literal"
    return TT_AddRigidBodies(rigid_bodies_file)

def load_project(project_file):
    """load project file"""
    assert type(project_file) is str, "Argument should be filename, i.e. a string literal"
    return TT_LoadProject(project_file)

def save_project(project_file):
    """save project file"""
    assert type(project_file) is str, "Argument should be filename, i.e. a string literal"
    return TT_SaveProject(project_file)

def load_calibration_from_memory(buffername,int buffersize):
    assert type (buffername) is str, "Argument should be buffername, i.e. a string literal"
    cdef unsigned char * buffer=buffername
    return TT_LoadCalibrationFromMemory(buffer,buffersize)

def update():
    """Process incoming camera data"""
    return TT_Update()

def update_single_frame():
    """Process incoming camera data"""
    return TT_UpdateSingleFrame()


#DATA STREAMING
def stream_trackd(bool enabled):
    """Start/stop Trackd Stream
       TrackD Streaming Engine: Streams rigid body data via the Trackd protocol"""
    return TT_StreamTrackd(enabled)

def stream_vrpn(bool enabled, int port):
    """Start/stop VRPN Stream
       VRPN Streaming Engine: Streams rigid body data via the VRPN protocol.
       VRPN Broadcast Port: Specifies the broadcast port for VRPN streaming. (Default: 3883)"""
    return TT_StreamVRPN(enabled, port)

def stream_np(bool enabled):
    """Start/stop NaturalPoint Stream"""
    return TT_StreamNP(enabled)


#FRAME
def frame_marker_count():
    """Returns Frame Markers Count"""
    return TT_FrameMarkerCount()

def frame_marker_x(int index):
    """Returns X Coord of Marker"""
    return TT_FrameMarkerX(index)

def frame_marker_y(int index):
    """Returns Y Coord of Marker"""
    return TT_FrameMarkerY(index)

def frame_marker_z(int index):
    """Returns Z Coord of Marker"""
    return TT_FrameMarkerZ(index)

def frame_marker_list():
    marker_list=[]
    for i in range(0,frame_marker_count()):
        marker_list.append(frame_marker_x(i))
        marker_list.append(frame_marker_y(i))
        marker_list.append(frame_marker_z(i))
    return marker_list

def frame_time_stamp():
    """Time Stamp of Frame (seconds"""
    return TT_FrameTimeStamp()

def frame_camera_centroid(int index, int cameraIndex, float x, float y):
    """Returns true if the camera is contributing to this 3D marker.
       It also returns the location of the 2D centroid that is reconstructing to this 3D marker"""
    if TT_FrameCameraCentroid(index,cameraIndex, x, y):
        print "\n \n 2D x-position as seen from camera %i is %f" % (cameraIndex, x)
        print "\n 2D y-position is %f" % y
    else:
        print "Camera is not contributing to the 3D position of this marker"

def camera_frame_buffer(int cameraIndex, int bufferPixelWidth, int bufferPixelHeight,
                        int bufferByteSpan, int bufferPixelBitDepth, buffername):
    """Fetch the camera's frame buffer.
    This function fills the provided buffer with an image of what is in the camera view.
    The resulting image depends on what video mode the camera is in.
    If the camera is in grayscale mode, for example, a grayscale image is returned from this call."""
    assert type (buffername) is str, "Argument should be buffername, i.e. a string literal"
    cdef unsigned char * buffer=buffername
    if TT_CameraFrameBuffer(cameraIndex,bufferPixelWidth,bufferPixelHeight,bufferByteSpan,bufferPixelBitDepth,buffer):
        print "Frame buffered"
    else:
        print "Error. Frame could not be buffered"

def camera_frame_buffer_save_as_bmp(int cameraIndex, str filename):
    """Save camera's frame buffer as a BMP image file"""
    assert type(filename) is str, "Argument should be filename, i.e. a string literal"
    if TT_CameraFrameBufferSaveAsBMP(cameraIndex, filename):
        print "Saved buffer as image file"
    else:
        print "Error. Could not save buffer"

def flush_camera_queues():
    """In the event that you are tracking a very high number of 2D and/or 3D markers (hundreds of 3D markers),
    and you find that the data you're getting out has sufficient latency you can call TT_FlushCameraQueues()
    to catch up before calling TT_Update(). Ideally, after calling TT_FlushCameraQueues() you'll want to not
    call it again until after TT_Update() returns 0"""
    TT_FlushCameraQueues()
    print "Flushed"


#RIGID BODY CONTROL
def set_rigid_body_user_data(int index, int ID):
    """Set RigidBodies User Data"""
    TT_SetRigidBodyUserData(index,ID)
    print "Set"

def rigid_body_user_data(int index):
    """Get RigidBodies User Data"""
    print "Rigid body ID: %i" %  TT_RigidBodyUserData(index)

def rigid_body_name(int index):
    """Returns RigidBody Name"""
    print "%s" % TT_RigidBodyName(index)

def set_rigid_body_enabled(int index, bool enabled):
    """Set tracking """
    TT_SetRigidBodyEnabled(index, enabled)

def rigid_body_enabled(int index):
    """Get tracking"""
    return TT_RigidBodyEnabled(index)

def is_rigid_body_tracked(int index):
    """Is rigid body currently tracked"""
    if TT_IsRigidBodyTracked(index):
        print "Yes"
    else:
        print "No"

def rigid_body_location(int index, float x, float y, float z,
                        float qx, float qy, float qz, float qw,
                        float yaw, float pitch, float roll):
    """##Not sure if this function sets or gets the location.
    If it returns values different from the ones you entered,
    the function gets the location as computed by Motive.
    Otherwise it is for manually setting the location.
    Update: So far this function only returns nonsense values.
            Maybe I have to initialize the variables in the function as c variables
            so that correct addresses are given to the function..."""
    TT_RigidBodyLocation(index,  &x, &y, &z,  &qx, &qy, &qz, &qw, &yaw, &pitch, &roll)
    print "The position of rigid body %i is x=%f, y=%f, z=%f. \n" % (index, x, y, z)
    print "Orientation in quaternions is qx=%f, qy=%f, qz=%f, qw=%f. \n" % (qx, qy, qz, qw)
    print "Yaw is %f, pitch is %f, roll is %f." % (yaw, pitch, roll)

def rigid_body_translate_pivot(int index, float x, float y, float z):
    """Rigid Body Pivot-Point Translation: Sets a translation offset for the centroid of the rigid body.
    Reported values for the location of the rigid body, as well as the 3D visualization, will be shifted
    by the amount provided in the fields on either the X, Y, or Z axis. Values are entered in meters. """
    return   TT_RigidBodyTranslatePivot(index, x, y, z)

def rigid_body_reset_orientation(int index):
    """Reset orientation to match the current tracked orientation
    of the rigid body"""
    TT_RigidBodyResetOrientation(index)

def clear_rigid_body_list():
    """Clear all rigid bodies"""
    TT_ClearRigidBodyList()
    print "Cleared"

def remove_rigid_body(int index):
    """Remove single rigid body"""
    return TT_RemoveRigidBody(index)

def rigid_body_marker_count(int index):
    """Get marker count"""
    return TT_RigidBodyMarkerCount(index)

def rigid_body_marker(int rigidIndex, int markerIndex, float x, float y, float z):
    """Get rigid body marker.
    ##Not sure if this function sets or gets the location.
    If it returns values different from the ones you entered,
    the function gets the location as computed by Motive.
    Otherwise it is for manually setting the location.
    Update: If the function only returns gibberish, see rigid body location."""
    TT_RigidBodyMarker(rigidIndex, markerIndex, &x, &y, &z)
    print "The position of rigid body's %i marker %i, is x=%f, y=%f, z=%f. \n" % (rigidIndex, markerIndex, x, y, z)

def rigid_body_point_cloud_marker(int rigidIndex, int markerIndex, bool tracked, float x, float y, float z):
    """ Get corresponding point cloud marker
    If tracked is false, there is no corresponding point cloud marker.
    """
    TT_RigidBodyPointCloudMarker(rigidIndex, markerIndex, tracked, x, y, z)
    if tracked:
        print "The point cloud markers position is x=%f, y=%f, z=%f" % (x,y,z)
    else:
        print "There is no corresponding point cloud marker"

def create_rigid_body(str name, int id, int markerCount, markerList):
    """Create a rigid body based on the marker count and marker list provided.
    The marker list is expected to contain a list of marker coordinates in the order:
    x1,y1,z1,x2,y2,z2,...xN,yN,zN."""
    cdef float markerListp[1000]
    assert len(markerList)<=1000, "Due to need of const C array size, markerList max items=1000. \n Please resize const in native.pyx"
    for i in range(0,len(markerList)):
        markerListp[i]=markerList[i]

    return TT_CreateRigidBody(name, id, markerCount, markerListp)

def software_build():
    """Software Release Build"""
    return TT_BuildNumber()


#CAMERA GROUP SUPPORT
def camera_group_count():
    """Returns number of camera groups"""
    return TT_CameraGroupCount()

def create_camera_group():
    """Add an additional group"""
    if TT_CreateCameraGroup():
        print "True"
    else:
        print "False"

def remove_camera_group(int index):
    """Remove a camera group (must be empty)"""
    if TT_RemoveCameraGroup(index):
        print "Removed"
    else:
        print "Error, could not remove. Check if group is empty"

def cameras_group(int index):
    """Returns Camera's camera group index"""
    return TT_CamerasGroup(index)

def set_group_shutter_delay(int groupIndex, int microseconds):
    """Set camera group's shutter delay"""
    TT_SetGroupShutterDelay(groupIndex, microseconds)
    print "Set"

def set_camera_group(int cameraIndex, int cameraGroupIndex):
    """Move camera to camera group"""
    TT_SetCameraGroup(cameraIndex, cameraGroupIndex)
    print "Set"

#MARKER SIZE SETTINGS
def set_camera_group_reconstruction(int groupIndex, bool enable):
    return TT_SetCameraGroupReconstruction(groupIndex, enable)

def set_enabled_filterswitch(bool enabled):
    return TT_SetEnabledFilterSwitch(enabled)

def is_filter_switch_enabled():
    if TT_IsFilterSwitchEnabled():
        print "True"
    else:
        print "False"

#POINT CLOUD INTERFACE
def camera_count():
    """Returns Camera Count"""
    return TT_CameraCount()

def camera_x_location(int index):
    """Returns Camera's X Coord"""
    return TT_CameraXLocation(index)

def camera_y_location(int index):
    """Returns Camera's Y Coord"""
    return TT_CameraYLocation(index)

def camera_z_location(int index):
    """Returns Camera's Z Coord"""
    return TT_CameraZLocation(index)

def camera_orientation_matrix(int camera, int index):
    """Orientation"""
    return TT_CameraOrientationMatrix(camera, index)

def camera_name(int index):
    """Returns Camera Name"""
    print TT_CameraName(index)

def camera_marker_count(int cameraIndex):
    """Camera's 2D Marker Count"""
    return TT_CameraMarkerCount(cameraIndex)

def camera_marker(int cameraIndex, int markerIndex, float x, float y):
    """CameraMarker fetches the 2D centroid location of the marker as seen by the camera"""
    if TT_CameraMarker(cameraIndex, markerIndex, x, y):
        print "The 2D location of marker %i is x=%f, y=%f" % (markerIndex, x, y)
    else:
        print "Error or no marker"

def camera_pixel_resolution(int cameraIndex, int width, int height):
    if TT_CameraPixelResolution(cameraIndex, width, height):
        print "Pixel resolution for camera %i is width=%f, height=%f" %(cameraIndex, width, height)
    else:
        print "Error"

def set_camera_settings(int camindex, int videotype, int exposure, int threshold, int intensity):
    """Set camera settings.  This function allows you to set the camera's video mode, exposure, threshold,
    and illumination settings.
    VideoType: 0 = Segment Mode, 1 = Grayscale Mode, 2 = Object Mode, 4 = Precision Mode, 6 = MJPEG Mode.
    Exposure: Valid values are:  1-480.
    Threshold: Valid values are: 0-255.
    Intensity: Valid values are: 0-15  (This should be set to 15 for most situations)"""
    return TT_SetCameraSettings(camindex, videotype, exposure, threshold, intensity)

def set_camera_frame_rate(int cameraIndex, int frameRate):
    """Set the frame rate for the given zero based camera index.
    Returns true if the operation was successful and false otherwise.
    If the operation fails, check that the camera index is valid and
    that devices have been initialized with TT_Initialize()"""
    if TT_SetCameraFrameRate(cameraIndex, frameRate):
        print "Set"
    else:
        print "Error. Not set"


#Get Camera Settings For A Given Camera Index. A Negative Return Value Indicates The Value Was Not
#Available. This Usually Means That Either The Camera Index Is Not Valid Or Devices Have Not Been
#Initialized With TT_Initialize()
def camera_frame_rate(int cameraIndex):
    """frames/sec"""
    return TT_CameraFrameRate(cameraIndex)

def camera_exposure(int cameraIndex):
    return TT_CameraExposure(cameraIndex)

def camera_threshold(int cameraIndex):
    return TT_CameraThreshold(cameraIndex)

def camera_intensity(int cameraIndex):
    return TT_CameraIntensity(cameraIndex)

def camera_temperature(int cameraIndex):
    return TT_CameraTemperature(cameraIndex)

def camera_ring_light_temperature(int cameraIndex):
    return  TT_CameraRinglightTemperature(cameraIndex)


#CAMERA'S FULL GRAYSCALE DECIMATION
def camera_grayscale_decimation(int cameraIndex):
    return  TT_CameraGrayscaleDecimation(cameraIndex)

def set_camera_grayscale_decimation(int cameraIndex, int value):
    if TT_SetCameraGrayscaleDecimation(cameraIndex, value):
        print "Set"
    else:
        print "Error. Not set"


#TOGGLE CAMERA EXTENDED OPTIONS
def set_camera_filter_switch(int cameraIndex, bool enableIRFilter):
    if TT_SetCameraFilterSwitch(cameraIndex,enableIRFilter):
        print "Set"
    else:
        print "Error. Possibly the camera does not have an IR filter"

def set_camera_agc(int cameraIndex, bool enableAutomaticGainControl):
    if TT_SetCameraAGC(cameraIndex, enableAutomaticGainControl):
        print "Set"
    else:
        print "Error. Possibly the camera does not have AGC"

def set_camera_aec(int cameraIndex, bool enableAutomaticExposureControl):
    if TT_SetCameraAEC(cameraIndex, enableAutomaticExposureControl):
        print "Set"
    else:
        print "Error. Possibly the camera does not have AEC"

def set_camera_high_power(int cameraIndex, bool enableHighPowerMode):
    if TT_SetCameraHighPower(cameraIndex, enableHighPowerMode):
        print "Set"
    else:
        print "Error. Possibly the camera does not have HighPowerMode"

def set_camera_mjpeg_high_quality(int cameraIndex, int mjpegQuality):
    if TT_SetCameraMJPEGHighQuality(cameraIndex, mjpegQuality):
        print "Set"
    else:
        print "Error. Possibly the camera does not have HighQuality for MJPEG"


#CAMERA IMAGER GAIN
def camera_imager_gain(int cameraIndex):
    return  TT_CameraImagerGain(cameraIndex)

def camera_imager_gain_levels(int cameraIndex):
    return  TT_CameraImagerGainLevels(cameraIndex)

def set_camera_imager_gain(int cameraIndex, int value):
    TT_SetCameraImagerGain(cameraIndex, value)
    print "Set"


#CAMERA ILLUMINATION
def is_continuous_ir_available(int cameraIndex):
    if TT_IsContinuousIRAvailable(cameraIndex):
        print "Yes"
    else:
        print "No"

def continuous_ir(int cameraIndex):
    if TT_ContinuousIR(cameraIndex):
        print "On"
    else:
        print "Off"

def set_continous_ir(int cameraIndex, bool enable):
    TT_SetContinuousIR(cameraIndex, enable)
    print "Set"

##def set_continuous_camera_mjpeg_high_quality_ir(int cameraIndex, bool Enable):
##    TT_SetContinuousTT_SetCameraMJPEGHighQualityIR(cameraIndex, Enable)
##    print "Set"


#CAMERA MASKING
def camera_mask(int cameraIndex, buffername, int bufferSize):
    assert type (buffername) is str, "Argument should be buffername, i.e. a string literal"
    cdef unsigned char * buffer=buffername
    if TT_CameraMask(cameraIndex, buffer, bufferSize):
        print "Mask exists"
    else:
        print "Mask does not exist"

def set_camera_mask(int cameraIndex, buffername, int bufferSize):
    assert type (buffername) is str, "Argument should be buffername, i.e. a string literal"
    cdef unsigned char * buffer=buffername
    if TT_SetCameraMask(cameraIndex, buffer, bufferSize):
        print "Set"
    else:
        print "Error. Not set"

def clear_camera_mask(int cameraIndex):
    if TT_ClearCameraMask(cameraIndex):
        print "Cleared"
    else:
        print "Not cleared"

def camera_mask_info(int cameraIndex, int blockingMaskWidth, int blockingMaskHeight, int blockingMaskGrid):
    if TT_CameraMaskInfo(cameraIndex, blockingMaskWidth, blockingMaskHeight, blockingMaskGrid):
        print "Camera %i blocking masks width:%f, height:%f, grid:%f" % (cameraIndex, blockingMaskWidth, blockingMaskHeight, blockingMaskGrid)
    else:
        print "Error. Possibly no mask for this camera"

#CAMERA ID
def camera_id(int cameraIndex):
    return TT_CameraID(cameraIndex)

#CAMERA DISTORTION
def camera_undistort_2d_point(int cameraIndex, float x, float y):
    """The 2D centroids the camera reports are distorted by the lens.  To remove the distortion call
    CameraUndistort2DPoint.  Also if you have a 2D undistorted point that you'd like to convert back
    to a distorted point call CameraDistort2DPoint."""
    TT_CameraUndistort2DPoint(cameraIndex, x, y)
    print "Undistorted"

def camera_distort_2d_point(int cameraIndex, float x, float y):
    """The 2D centroids the camera reports are distorted by the lens.  To remove the distortion call
    CameraUndistort2DPoint.  Also if you have a 2D undistorted point that you'd like to convert back
    to a distorted point call CameraDistort2DPoint."""
    TT_CameraDistort2DPoint(cameraIndex, x, y)
    print "Distort"

def camera_ray(int cameraIndex, float x, float y,
               float rayStartX, float rayStartY, float rayStartZ,
               float rayEndX,   float rayEndY,   float rayEndZ):
    """Takes an undistorted 2D centroid and return a camera ray in the world coordinate system."""
    if TT_CameraRay(cameraIndex, x, y, rayStartX, rayStartY, rayStartZ, rayEndX, rayEndY, rayEndZ):
        print "Ray Xstart=%f, Xend=%f, Ystart=%f, Yend=%f, Zstart=%f, Zend=%f" % (rayStartX, rayStartY, rayStartZ, rayEndX, rayEndY, rayEndZ)
    else:
        print "Error"

def camera_model(int cameraIndex, float x, float y, float z,                   #Camera Position
                                  orientation,                                 #Orientation (3x3 matrix)
                                  float principleX, float principleY,          #Lens center (in pixels)
                                  float focalLengthX, float focalLengthY,      #Lens focal  (in pixels)
                                  float kc1, float kc2, float kc3,             #Barrel distortion coefficients
                                  float tangential0, float tangential1):       #Tangential distortion
    """Set a camera's extrinsic (position & orientation) and intrinsic (lens distortion) parameters
    with parameters compatible with the OpenCV intrinsic model. """
    cdef float orientationp[9]
    for i in range(0,9):
        orientationp[i]=orientation[i]
    if TT_CameraModel(cameraIndex, x, y, z,orientationp,principleX, principleY,
                     focalLengthX, focalLengthY, kc1, kc2, kc3, tangential0, tangential1):
       print "Set"
    else:
       print "Error. Could not set parameters"

#ADDITIONAL FUNCTIONALITY
def camera_backproject(int cameraIndex, float x, float y, float z, float cameraX, float cameraY):
    """Back-project from 3D space to 2D space.  If you give this function a 3D location and select a camera,
       it will return where the point would land on the imager of that camera in to 2D space.
       This basically locates where in the camera's FOV a 3D point would be located.
    """
    TT_CameraBackproject(cameraIndex, x, y, z, cameraX, cameraY)
    print "Point in camera 2D space: x=%f, y=%f" % (cameraX, cameraY)

def set_frame_id_based_timing(bool enable):
    if TT_SetFrameIDBasedTiming(enable):
        print "Set"
    else:
        print "Not set"

def set_suppress_out_of_order(bool enable):
    if TT_SetSuppressOutOfOrder(enable):
        print "Set"
    else:
        print "Not set"

def orient_tracking_bar(float positionX, float positionY, float positionZ,
                        float orientationX, float orientationY, float orientationZ, float orientationW):
    return TT_OrientTrackingBar(positionX, positionY, positionZ,
                                orientationX, orientationY, orientationZ, orientationW)



