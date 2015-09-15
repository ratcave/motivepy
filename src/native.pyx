__author__ = 'Vash'

# means comment from official SDK API (or Cython page) ## means comment from me

include "cnative.pxd"

def check_npresult(func):
    def wrapper(*args, **kwargs):
        npresult = func(*args, **kwargs)
        if npresult == 1:
            raise IOError("File Not Found")
        elif npresult == 2:
            raise IOError("Load Failed")
        elif npresult == 3:
            raise Exception("Failed")
        elif npresult == 8:
            raise IOError("Invalid File")
        elif npresult == 9:
            raise IOError("Invalid Calibration File")
        elif npresult == 10:
            raise EnvironmentError("Unable To Initialize")
        elif npresult == 11:
            raise EnvironmentError("Invalid License")
        elif npresult == 14:
            raise RuntimeWarning("No Frames Available")   #think about putting a try statement here, which tries to get the next frame for a couple of times for example
    return wrapper

def check_cam_setting(func):
    def wrapper(*args, **kwargs):
        check=func(*args, **kwargs)
        if check<0:
            raise Exception("Value Not Available. Usually Camera Index Not Valid Or Devices Not Initialized")
        else:
            return check
    return wrapper


#STARTUP / SHUTDOWN
@check_npresult
def initialize():
    """initialize library"""
    return TT_Initialize()

@check_npresult
def shutdown():
    """shutdown library"""
    return TT_Shutdown()

#RIGID BODY INTERFACE ##FILES
@check_npresult
def load_calibration(str calib_filename):
    """load calibration"""
    return TT_LoadCalibration(calib_filename)

@check_npresult
def load_rigid_bodies(str load_bodies_file):
    """load rigid bodies"""
    return TT_LoadRigidBodies(load_bodies_file)

@check_npresult
def save_rigid_bodies(str rigid_filename):
    """save rigid bodies"""
    return TT_SaveRigidBodies(rigid_filename)

@check_npresult
def add_rigid_bodies(str rigid_bodies_file):
    """add rigid bodies"""
    return TT_AddRigidBodies(rigid_bodies_file)

@check_npresult
def load_project(str project_file):
    """load project file"""
    return TT_LoadProject(project_file)

@check_npresult
def save_project(str project_file):
    """save project file"""
    return TT_SaveProject(project_file)

@check_npresult
def load_calibration_from_memory(buffername,int buffersize):
    assert isinstance(buffername,str), "Buffername Needs To Be String"
    cdef unsigned char * buffer=buffername
    return TT_LoadCalibrationFromMemory(buffer,buffersize)

@check_npresult
def update():
    """Process incoming camera data"""
    return TT_Update()

@check_npresult
def update_single_frame():
    """Process incoming camera data"""
    return TT_UpdateSingleFrame()


#DATA STREAMING
@check_npresult
def stream_trackd(bool enabled):
    """Start/stop Trackd Stream
       TrackD Streaming Engine: Streams rigid body data via the Trackd protocol"""
    return TT_StreamTrackd(enabled)

@check_npresult
def stream_vrpn(bool enabled, int port):
    """Start/stop VRPN Stream
       VRPN Streaming Engine: Streams rigid body data via the VRPN protocol.
       VRPN Broadcast Port: Specifies the broadcast port for VRPN streaming. (Default: 3883)"""
    return TT_StreamVRPN(enabled, port)

@check_npresult
def stream_np(bool enabled):
    """Start/stop NaturalPoint Stream"""
    return TT_StreamNP(enabled)


#FRAME
def frame_marker_count():
    """Returns Frame Markers Count"""
    return TT_FrameMarkerCount()

def frame_marker_x(int markerIndex):
    """Returns X Coord of Marker"""
    return TT_FrameMarkerX(markerIndex)

def frame_marker_y(int markerIndex):
    """Returns Y Coord of Marker"""
    return TT_FrameMarkerY(markerIndex)

def frame_marker_z(int markerIndex):
    """Returns Z Coord of Marker"""
    return TT_FrameMarkerZ(markerIndex)

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

def frame_camera_centroid(int markerIndex, int cameraIndex, float x, float y):
    """Returns true if the camera is contributing to this 3D marker.
       It also returns the location of the 2D centroid that is reconstructing to this 3D marker"""
    if TT_FrameCameraCentroid(markerIndex,cameraIndex, x, y):
        print "Camera {0} sees 2D x-position={1}, y-position={2}".format(cameraIndex, x, y)
    else:
        raise Exception("Camera Not Contributing To 3D Position Of Marker {0}. \n Try Different Camera.".format(markerIndex))

def camera_frame_buffer(int cameraIndex, int bufferPixelWidth, int bufferPixelHeight,
                        int bufferByteSpan, int bufferPixelBitDepth, buffername):
    """Fetch the camera's frame buffer.
    This function fills the provided buffer with an image of what is in the camera view.
    The resulting image depends on what video mode the camera is in.
    If the camera is in grayscale mode, for example, a grayscale image is returned from this call."""
    assert isinstance(buffername,str), "Buffername Needs To Be String"
    cdef unsigned char * buffer=buffername
    if not TT_CameraFrameBuffer(cameraIndex,bufferPixelWidth,bufferPixelHeight,bufferByteSpan,bufferPixelBitDepth,buffer):
        raise BufferError("Camera Frame Could Not Be Buffered")

def camera_frame_buffer_save_as_bmp(int cameraIndex, str filename):
    """Save camera's frame buffer as a BMP image file"""
    if not TT_CameraFrameBufferSaveAsBMP(cameraIndex, filename):
        raise IOError("Camera Frame Buffer Not Successfully Saved To Filename: {0}.".format(filename))

def flush_camera_queues():
    """In the event that you are tracking a very high number of 2D and/or 3D markers (hundreds of 3D markers),
    and you find that the data you're getting out has sufficient latency you can call TT_FlushCameraQueues()
    to catch up before calling TT_Update(). Ideally, after calling TT_FlushCameraQueues() you'll want to not
    call it again until after TT_Update() returns 0"""
    TT_FlushCameraQueues()


#CAMERA GROUP SUPPORT
def camera_group_count():
    """Returns number of camera groups"""
    return TT_CameraGroupCount()

def create_camera_group():
    """Add an additional group"""
    if not TT_CreateCameraGroup():
        print Exception("Could Not Create Camera Group")

def remove_camera_group(int groupIndex):
    """Remove a camera group (must be empty)"""
    if not TT_RemoveCameraGroup(groupIndex):
        raise Exception("Could Not Remove. Check If Group Empty")

def cameras_group(int cameraIndex):
    """Returns Camera's camera group index"""
    return TT_CamerasGroup(cameraIndex)

def set_group_shutter_delay(int groupIndex, int microseconds):
    """Set camera group's shutter delay"""
    TT_SetGroupShutterDelay(groupIndex, microseconds)

def set_camera_group(int cameraIndex, int groupIndex):
    """Move camera to camera group"""
    TT_SetCameraGroup(cameraIndex, groupIndex)


#RIGID BODY CONTROL
def set_rigid_body_user_data(int rigidIndex, int ID):
    """Set RigidBodies User Data"""
    TT_SetRigidBodyUserData(rigidIndex,ID)

def rigid_body_user_data(int rigidIndex):
    """Get RigidBodies User Data"""
    print "Rigid body ID: {0}".format(TT_RigidBodyUserData(rigidIndex))

def rigid_body_name(int rigidIndex):
    """Returns RigidBody Name"""
    print "{0}".format(TT_RigidBodyName(rigidIndex))

def set_rigid_body_enabled(int rigidIndex, bool enabled):
    """Set tracking """
    TT_SetRigidBodyEnabled(rigidIndex, enabled)

def rigid_body_enabled(int rigidIndex):
    """Get tracking"""
    return TT_RigidBodyEnabled(rigidIndex)

def is_rigid_body_tracked(int rigidIndex):
    """Is rigid body currently tracked"""
    return TT_IsRigidBodyTracked(rigidIndex)

def rigid_body_location(int rigidIndex, float x, float y, float z,
                        float qx, float qy, float qz, float qw,
                        float yaw, float pitch, float roll):
    """##Not sure if this function sets or gets the location.
    If it returns values different from the ones you entered,
    the function gets the location as computed by Motive.
    Otherwise it is for manually setting the location.
    Update: So far this function only returns nonsense values.
            Maybe I have to initialize the variables in the function as c variables
            so that correct addresses are given to the function..."""
    TT_RigidBodyLocation(rigidIndex,  &x, &y, &z,  &qx, &qy, &qz, &qw, &yaw, &pitch, &roll)
    print "The position of rigid body {0} is x={1}, y={2}, z={3}. \n".format(rigidIndex, x, y, z)
    print "Orientation in quaternions is qx={0}, qy={1}, qz={2}, qw={3}. \n".format(qx, qy, qz, qw)
    print "Yaw is {0}, pitch is {1}, roll is {2}.".format(yaw, pitch, roll)

@check_npresult
def rigid_body_translate_pivot(int rigidIndex, float x, float y, float z):
    """Rigid Body Pivot-Point Translation: Sets a translation offset for the centroid of the rigid body.
    Reported values for the location of the rigid body, as well as the 3D visualization, will be shifted
    by the amount provided in the fields on either the X, Y, or Z axis. Values are entered in meters. """
    return   TT_RigidBodyTranslatePivot(rigidIndex, x, y, z)

def rigid_body_reset_orientation(int rigidIndex):
    """Reset orientation to match the current tracked orientation
    of the rigid body"""
    TT_RigidBodyResetOrientation(rigidIndex)

def clear_rigid_body_list():
    """Clear all rigid bodies"""
    TT_ClearRigidBodyList()

@check_npresult
def remove_rigid_body(int rigidIndex):
    """Remove single rigid body"""
    return TT_RemoveRigidBody(rigidIndex)

def rigid_body_marker_count(rigidIndex):
    """Get marker count"""
    return TT_RigidBodyMarkerCount(rigidIndex)

def rigid_body_marker(int rigidIndex, int markerIndex, float x, float y, float z):
    """Get rigid body marker.
    ##Not sure if this function sets or gets the location.
    If it returns values different from the ones you entered,
    the function gets the location as computed by Motive.
    Otherwise it is for manually setting the location.
    Update: If the function only returns gibberish, see rigid body location."""
    TT_RigidBodyMarker(rigidIndex, markerIndex, &x, &y, &z)
    print "The position of rigid body's {0} marker {1}, is x={2}, y={3}, z={4}. \n".format(rigidIndex, markerIndex, x, y, z)

def rigid_body_point_cloud_marker(int rigidIndex, int markerIndex, bool tracked, float x, float y, float z):
    """ Get corresponding point cloud marker
    If tracked is false, there is no corresponding point cloud marker.
    """
    TT_RigidBodyPointCloudMarker(rigidIndex, markerIndex, tracked, x, y, z)
    if tracked:
        print "The point cloud markers position is x={0}, y={1}, z={2}".format(x,y,z)
    else:
        raise Exception("No Corresponding Point Cloud Marker")

@check_npresult
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


#MARKER SIZE SETTINGS
@check_npresult
def set_camera_group_reconstruction(int groupIndex, bool enable):
    return TT_SetCameraGroupReconstruction(groupIndex, enable)

@check_npresult
def set_enabled_filterswitch(bool enabled):
    return TT_SetEnabledFilterSwitch(enabled)

def is_filter_switch_enabled():
    return TT_IsFilterSwitchEnabled()


#POINT CLOUD INTERFACE ##CAMERA INTERFACE
def camera_count():
    """Returns Camera Count"""
    return TT_CameraCount()


class Camera(object):
    def __init__(self, cameraIndex):
        self.index=cameraIndex

    @property
    @check_cam_setting
    def frame_rate(self):
        """frames/sec (int)"""
        return TT_CameraFrameRate(self.index)

    @frame_rate.setter
    def frame_rate(self, value):
        if not TT_SetCameraFrameRate(self.index, value):
            raise Exception("Could Not Set Frame Rate. Check Camera Index And Initialize With TT_Initialize()")

    @property
    @check_cam_setting
    def grayscale_decimation(int cameraIndex):
        return  TT_CameraGrayscaleDecimation(cameraIndex)

    @grayscale_decimation.setter
    def grayscale_decimation(self, value):
        if not TT_SetCameraGrayscaleDecimation(self.index, value):
            raise Exception("Could Not Set Decimation")

    @property
    @check_cam_setting
    def imager_gain(self):
        return  TT_CameraImagerGain(self.index)

    @imager_gain.setter
    def imager_gain(self, value):
        levels=TT_CameraImagerGainLevels(self.value)
        assert value<=levels, "Maximum Gain Level is {0}".format(levels)
        TT_SetCameraImagerGain(self.index, value)

    @property
    def continuous_ir(self):
        assert TT_IsContinuousIRAvailable(self.index), "Camera {0} Does Not Support Continuous IR".format(self.index)
        return TT_ContinuousIR(self.index)

    @continuous_ir.setter
    def continuous_ir(self, bool value):
        TT_SetContinuousIR(self.index, value)

##def set_continuous_camera_mjpeg_high_quality_ir(int cameraIndex, bool Enable):
##    TT_SetContinuousTT_SetCameraMJPEGHighQualityIR(cameraIndex, Enable)
##    print "Set"

#Properties Without Simple Setter (If Not Here Maybe In Camera Class)
    @property
    def name(self):
        """Camera Name (str)"""
        return TT_CameraName(self.index)

    @property
    @check_cam_setting
    def id(self):
        return TT_CameraID(self.index)

    @property
    def x_location(self):
        """Returns Camera's X Coord"""
        return TT_CameraXLocation(self.index)

    @property
    def y_location(self):
        """Returns Camera's Y Coord"""
        return TT_CameraYLocation(self.index)

    @property
    def z_location(self):
        """Returns Camera's Z Coord"""
        return TT_CameraZLocation(self.index)

    def model(self, int cameraIndex, float x, float y, float z,  #Camera Position
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
        if not TT_CameraModel(cameraIndex, x, y, z, orientationp, principleX, principleY,
                              focalLengthX, focalLengthY, kc1, kc2, kc3, tangential0, tangential1):
            raise Exception("Could Not Set Parameters")

    @property
    @check_cam_setting
    def video_type(self):
        return TT_CameraVideoType(self.index)

    @property
    @check_cam_setting
    def exposure(self):
        return TT_CameraExposure(self.index)

    @property
    @check_cam_setting
    def threshold(self):
        return TT_CameraThreshold(self.index)

    @property
    @check_cam_setting
    def intensity(self):
        return TT_CameraIntensity(self.index)

    def set_settings(self, int videotype, int exposure, int threshold, int intensity):
        """Set camera settings.  This function allows you to set the camera's video mode, exposure, threshold,
        and illumination settings.
        VideoType: 0 = Segment Mode, 1 = Grayscale Mode, 2 = Object Mode, 4 = Precision Mode, 6 = MJPEG Mode.
        Exposure: Valid values are:  1-480.
        Threshold: Valid values are: 0-255.
        Intensity: Valid values are: 0-15  (This should be set to 15 for most situations)"""
        return TT_SetCameraSettings(self.index, videotype, exposure, threshold, intensity)

    @property
    @check_cam_setting
    def imager_gain_levels(self):
        return  TT_CameraImagerGainLevels(self.index)

    @property
    def is_continuous_ir_available(self):
        return TT_IsContinuousIRAvailable(self.index)

    @property
    @check_cam_setting
    def temperature(self):
        return TT_CameraTemperature(self.index)

    @property
    @check_cam_setting
    def ring_light_temperature(self):
        return  TT_CameraRinglightTemperature(self.index)



    @property
    def marker_count(self):
        """Camera's 2D Marker Count"""
        return TT_CameraMarkerCount(self.index)


#Functions To Set Camera Property Value, But W\O Possibility To Get Value

    def set_filter_switch(self, bool enableIRFilter):
        if not TT_SetCameraFilterSwitch(self.index, enableIRFilter):
            raise Exception("Could Not Switch Filter. Possibly Camera Has No IR Filter")

    def set_agc(self, bool enableAutomaticGainControl):
        if not TT_SetCameraAGC(self.index, enableAutomaticGainControl):
            raise Exception("Could Not Enable AGC. Possibly Camera Has No AGC")

    def set_aec(self, bool enableAutomaticExposureControl):
        if not TT_SetCameraAEC(self.index, enableAutomaticExposureControl):
            raise Exception("Could Not Enable AEC. Possibly Camera Has No AEC")

    def set_high_power(self, bool enableHighPowerMode):
        if not TT_SetCameraHighPower(self.index, enableHighPowerMode):
            raise Exception("Could Not Enable HighPowerMode. Possibly Camera Has No HighPowerMode")

    def set_mjpeg_high_quality(self, int mjpegQuality):
        if not TT_SetCameraMJPEGHighQuality(self.index, mjpegQuality):
            raise Exception("Could Not Enable HighQuality. Possibly Camera Has No HighQuality For MJPEG")


def orientation_matrix(int cameraIndex, int matrixIndex):
    """Orientation"""
    return TT_CameraOrientationMatrix(cameraIndex, matrixIndex)


def camera_marker(int cameraIndex, int markerIndex, float x, float y):
    """CameraMarker fetches the 2D centroid location of the marker as seen by the camera"""
    if TT_CameraMarker(cameraIndex, markerIndex, x, y):
        print "The 2D location of marker {0} is x={1}, y={2}".format(markerIndex, x, y)
    else:
        raise Exception("Could Not Fetch Location. Possibly Marker {0} Is Not Seen By Camera".format(markerIndex))

def camera_pixel_resolution(int cameraIndex, int width, int height):
    if TT_CameraPixelResolution(cameraIndex, width, height):
        print "Pixel resolution for camera {0} is width={1}, height={2}".format(cameraIndex, width, height)
    else:
        raise Exception

#CAMERA MASKING
def camera_mask(int cameraIndex, buffername, int bufferSize):
    assert isinstance(buffername,str), "Buffername Needs To Be String"
    cdef unsigned char * buffer=buffername
    return TT_CameraMask(cameraIndex, buffer, bufferSize)


def set_camera_mask(int cameraIndex, buffername, int bufferSize):
    assert isinstance(buffername,str), "Buffername Needs To Be String"
    cdef unsigned char * buffer=buffername
    if not TT_SetCameraMask(cameraIndex, buffer, bufferSize):
        raise Exception("Could Not Set Mask")

def clear_camera_mask(int cameraIndex):
    if not TT_ClearCameraMask(cameraIndex):
        raise Exception("Could Not Clear Mask")

def camera_mask_info(int cameraIndex, int blockingMaskWidth, int blockingMaskHeight, int blockingMaskGrid):
    if TT_CameraMaskInfo(cameraIndex, blockingMaskWidth, blockingMaskHeight, blockingMaskGrid):
        print "Camera {0} blocking masks width:{1}, height:{2}, grid:{3}".format(cameraIndex, blockingMaskWidth, blockingMaskHeight, blockingMaskGrid)
    else:
        raise Exception("Possibly Camera {0} Has No Mask".format(cameraIndex))



#CAMERA DISTORTION
def camera_undistort_2d_point(int cameraIndex, float x, float y):
    """The 2D centroids the camera reports are distorted by the lens.  To remove the distortion call
    CameraUndistort2DPoint.  Also if you have a 2D undistorted point that you'd like to convert back
    to a distorted point call CameraDistort2DPoint."""
    TT_CameraUndistort2DPoint(cameraIndex, x, y)

def camera_distort_2d_point(int cameraIndex, float x, float y):
    """The 2D centroids the camera reports are distorted by the lens.  To remove the distortion call
    CameraUndistort2DPoint.  Also if you have a 2D undistorted point that you'd like to convert back
    to a distorted point call CameraDistort2DPoint."""
    TT_CameraDistort2DPoint(cameraIndex, x, y)

def camera_ray(int cameraIndex, float x, float y,
               float rayStartX, float rayStartY, float rayStartZ,
               float rayEndX,   float rayEndY,   float rayEndZ):
    """Takes an undistorted 2D centroid and return a camera ray in the world coordinate system."""
    if TT_CameraRay(cameraIndex, x, y, rayStartX, rayStartY, rayStartZ, rayEndX, rayEndY, rayEndZ):
        print "Ray Xstart={0}, Xend={1}, Ystart={2}, Yend={3}, Zstart={4}, Zend={5}".format(rayStartX, rayStartY, rayStartZ, rayEndX, rayEndY, rayEndZ)
    else:
        raise Exception



#ADDITIONAL FUNCTIONALITY
def camera_backproject(int cameraIndex, float x, float y, float z, float cameraX, float cameraY):
    """Back-project from 3D space to 2D space.  If you give this function a 3D location and select a camera,
       it will return where the point would land on the imager of that camera in to 2D space.
       This basically locates where in the camera's FOV a 3D point would be located.
    """
    TT_CameraBackproject(cameraIndex, x, y, z, cameraX, cameraY)
    print "Point in camera 2D space: x={0}, y={1}".format(cameraX, cameraY)

def set_frame_id_based_timing(bool enable):
    return TT_SetFrameIDBasedTiming(enable)

def set_suppress_out_of_order(bool enable):
    return TT_SetSuppressOutOfOrder(enable)

@check_npresult
def orient_tracking_bar(float positionX, float positionY, float positionZ,
                        float orientationX, float orientationY, float orientationZ, float orientationW):
    return TT_OrientTrackingBar(positionX, positionY, positionZ,
                                orientationX, orientationY, orientationZ, orientationW)



