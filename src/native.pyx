__author__ = 'Vash'

# means comment from official SDK API (or Cython page) ## means comment from me

include "cnative.pxd"

def check_npresult(func):
    """Checks if the output of a function matches the Motive Error Values, and raises a Python error if so."""
    error_dict = {1: (IOError, "File Not Found"),
                  2: (Exception, "Load Failed"),
                  3: (Exception, "Failed"),
                  8: (IOError, "Invalid File"),
                  9: (IOError, "Invalid Calibration File"),
                  10: (EnvironmentError, "Unable To Initialize"),
                  11: (EnvironmentError, "Invalid License"),
                  14: (RuntimeWarning, "No Frames Available")}

    def wrapper(*args, **kwargs):
        npresult = func(*args, **kwargs)
        if npresult in error_dict:
            error, msg = error_dict[npresult]
            raise error(msg)

    return wrapper


def block_for_frame(secs_to_timeout=1):
    """Decorator to Continually calls a function until it stops raising a RuntimeWarning until timeout."""
    import time
    def decorator_fun(func):
        def wrapper(, *args, **kwargs):

            end_time = time.time() + secs_to_timeout
            while time.time() < end_time:
                try:
                    output = func(*args, **kwargs)
                    break
                except RuntimeWarning:
                    pass
            else:
                raise RuntimeWarning("Timed Out after {} seconds".format(secs_to_timeout))
            return output
        return wrapper
    return decorator_fun




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

@block_for_frame(secs_to_timeout=1)
@check_npresult
def update():
    """Process incoming camera data."""
    return TT_Update()

@block_for_frame(secs_to_timeout=1)
@check_npresult
def update_single_frame():
    """Process incoming camera data"""
    return TT_UpdateSingleFrame()


#DATA STREAMING
@check_npresult
def stream_trackd(bool enabled):
    """Start/stop Trackd Stream.
       TrackD Streaming Engine: Streams rigid body data via the Trackd protocol"""
    return TT_StreamTrackd(enabled)

@check_npresult
def stream_vrpn(bool enabled, int port=3883):
    """Start/stop VRPN Stream.
       VRPN Streaming Engine: Streams rigid body data via the VRPN protocol.
       VRPN Broadcast Port: Specifies the broadcast port for VRPN streaming."""
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

def frame_markers():
    markers=[]
    for i in range(0,frame_marker_count()):
        nest_markers=[]
        nest_markers.append(frame_marker_x(i))
        nest_markers.append(frame_marker_y(i))
        nest_markers.append(frame_marker_z(i))
        markers.append(nest_markers)
    return markers

def unident_markers(int rigidBody_count):
    """
    returns a list of all markers which
    are not in rigid Bodies
    """
    markers=frame_markers()
    unimarkers=[]
    imarkers=[]
    for i in range (0,rigidBody_count):
        for ik in rigidBody_markers(i):
           imarkers.append(ik)
           print "{}\n".format(ik)
        print '\n{}\n\n'.format(imarkers)
    for k in markers:
        if k not in imarkers:
           unimarkers.append(k)
           print '{}\n'.format(k)
    return unimarkers



def frame_time_stamp():
    """Time Stamp of Frame (seconds)"""
    return TT_FrameTimeStamp()

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
        raise Exception("Could Not Create Camera Group")

def remove_camera_group(int groupIndex):
    """Remove a camera group (must be empty)"""
    if not TT_RemoveCameraGroup(groupIndex):
        raise Exception("Could Not Remove. Check If Group Empty")

def set_group_shutter_delay(int groupIndex, int microseconds):
    """Set camera group's shutter delay"""
    TT_SetGroupShutterDelay(groupIndex, microseconds)


#RIGID BODY CONTROL
rigidBodyCount=0

def rigidBody_count():
    """returns number of rigid bodies"""
    return rigidBodyCount

@check_npresult
def create_rigid_body(str name, markerList):
     """
     The marker list is expected to contain a list of marker coordinates in the order:
     x1,y1,z1,x2,y2,z2,...xN,yN,zN.
     """
     id = rigidBody_count()-1
     cdef float markerListp[1000]
     markerCount=len(markerList)
     assert markerCount<=1000, "Due to need of const C array size, markerList max items=1000. \n Please resize const in native.pyx"
     for i in range(0,len(markerList)):
         markerListp[3*i]=markerList[i][0]
         markerListp[3*i+1]=markerList[i][1]
         markerListp[3*i+2]=markerList[i][2]
     if (TT_CreateRigidBody(name, id, markerCount, markerListp)==0):
         global rigidBodyCount
         rigidBodyCount=rigidBodyCount+1
     return TT_CreateRigidBody(name, id, markerCount, markerListp)

@check_npresult
def remove_rigid_body(int rigidIndex):
    """Remove single rigid body"""
    if (TT_RemoveRigidBody(rigidIndex)==0):
        global rigidBodyCount
        rigidBodyCount=rigidBodyCount-1
    return TT_RemoveRigidBody(rigidIndex)

def clear_rigid_body_list():
        """Clear all rigid bodies"""
        TT_ClearRigidBodyList()
        global rigidBodyCount
        rigidBodyCount=0


class RigidBody(object):
    def __init__(self, rigidIndex):
        #assert 0<=rigidIndex<rigidBodyCount, "There Are Only {0} Rigid Bodies".format(rigidBodyCount)
        self.index=rigidIndex

    @property
    def user_data(self):
        """Get RigidBodies User Data"""
        return "Rigid body ID: {0}".format(TT_RigidBodyUserData(self.index))

    @user_data.setter
    def user_data(self,value):
        """ (int index) """
        return TT_SetRigidBodyUserData(self.index,value)

    @property
    def tracking_enabled(self):
        """Get tracking (bool)"""
        return TT_RigidBodyEnabled(self.index)

    @tracking_enabled.setter
    def tracking_enabled(self, value):
        TT_SetRigidBodyEnabled(self.index, value)

#Properties Without Simple Setter (If Not Here Maybe In Camera Class)
    @property
    def name(self):
        """Returns RigidBody Name"""
        return "{0}".format(TT_RigidBodyName(self.index))

    @property
    def is_tracked(self):
        """Is rigid body currently tracked"""
        return TT_IsRigidBodyTracked(self.index)

    @property
    def marker_count(self):
        """Get marker count"""
        return TT_RigidBodyMarkerCount(self.index)

    def point_cloud_marker(self, int markerIndex):
        """ Get corresponding point cloud marker
        If tracked is false, there is no corresponding point cloud marker.
        """
        cdef bool tracked=True
        cdef float x=0
        cdef float y=0
        cdef float z=0
        TT_RigidBodyPointCloudMarker(self.index, markerIndex, tracked, x, y, z)
        if tracked:
            return "The point cloud markers position is x={0}, y={1}, z={2}".format(x,y,z)
        else:
            raise Exception("No Corresponding Point Cloud Marker")

    def location(self, float x, float y, float z,
                       float qx, float qy, float qz, float qw,
                       float yaw, float pitch, float roll):
        """##
        Function returns location of rigid body.
        So far it seems necessary to load the rigid body data
        through first loading project file."""
        TT_RigidBodyLocation(self.index,  &x, &y, &z,  &qx, &qy, &qz, &qw, &yaw, &pitch, &roll)
        return "The position of rigid body {0} is x={1}, y={2}, z={3}. \n".format(self.index, x, y, z)
        return "Orientation in quaternions is qx={0}, qy={1}, qz={2}, qw={3}. \n".format(qx, qy, qz, qw)
        return "Yaw is {0}, pitch is {1}, roll is {2}.".format(yaw, pitch, roll)

    def marker(self, int markerIndex):
        """Get rigid body marker.
        ##This function gets the location.
        """
        cdef float x=0
        cdef float y=0
        cdef float z=0
        TT_RigidBodyMarker(self.index, markerIndex, &x, &y, &z)
        return "The position of rigid body {0} marker {1}, is x={2}, y={3}, z={4}. \n".format(TT_RigidBodyName(self.index), markerIndex, x, y, z)

    def markers(self):
        """Get list of rigid body marker position"""
        mcount=TT_RigidBodyMarkerCount(self.index)
        markers=[]
        cdef float x=0
        cdef float y=0
        cdef float z=0
        for i in range(0,mcount):
            TT_RigidBodyMarker(self.index, i, &x, &y, &z)
            nest_markers=[x, y, z]
            # nest_markers.append(x)
            # nest_markers.append(y)
            # nest_markers.append(z)
            markers.append(nest_markers)
        return markers


    @check_npresult
    def translate_pivot(self, float x, float y, float z):
        """Rigid Body Pivot-Point Translation: Sets a translation offset for the centroid of the rigid body.
        Reported values for the location of the rigid body, as well as the 3D visualization, will be shifted
        by the amount provided in the fields on either the X, Y, or Z axis. Values are entered in meters. """
        return   TT_RigidBodyTranslatePivot(self.index, x, y, z)

    def reset_orientation(self):
        """Reset orientation to match the current tracked orientation
        of the rigid body"""
        TT_RigidBodyResetOrientation(self.index)

def rigidBody_markers(rigidIndex):
    """Get list of any rigid body marker position"""
    mcount=TT_RigidBodyMarkerCount(rigidIndex)
    markers=[]
    cdef float x=0
    cdef float y=0
    cdef float z=0
    for i in range(0,mcount):
        TT_RigidBodyMarker(rigidIndex, i, &x, &y, &z)
        nest_markers=[]
        nest_markers.append(x)
        nest_markers.append(y)
        nest_markers.append(z)
        markers.append(nest_markers)
    return markers


#MARKER SIZE SETTINGS
@check_npresult
def set_camera_group_reconstruction(int groupIndex, bool enable):
    return TT_SetCameraGroupReconstruction(groupIndex, enable)

@check_npresult
def set_enabled_filter_switch(bool enabled):
    return TT_SetEnabledFilterSwitch(enabled)

def is_filter_switch_enabled():
    return TT_IsFilterSwitchEnabled()


#POINT CLOUD INTERFACE ##CAMERA INTERFACE
def camera_count():
    """Returns Camera Count"""
    return TT_CameraCount()

def return_cams():
    """Initiate all cameras as python objects,
    where camera #k is cam[k-1]"""
    cam=[Camera(cameraIndex) for cameraIndex in range(camera_count())]
    return cam


class Camera(object):
    def __init__(self, cameraIndex):
        assert cameraIndex<camera_count(), "There Are Only {0} Cameras".format(camera_count())
        self.index=cameraIndex

    @property
    def group(self):
        """Camera's camera group index (int)"""
        return TT_CamerasGroup(self.index)

    @group.setter
    def group(self,value):
        return TT_SetCameraGroup(self.index, value)

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
    def grayscale_decimation(self):
        """ (int) """
        return  TT_CameraGrayscaleDecimation(self.index)

    @grayscale_decimation.setter
    def grayscale_decimation(self, value):
        if not TT_SetCameraGrayscaleDecimation(self.index, value):
            raise Exception("Could Not Set Decimation")

    @property
    @check_cam_setting
    def imager_gain(self):
        """ (int)
        """
        return  TT_CameraImagerGain(self.index)

    @imager_gain.setter
    def imager_gain(self, value):
        assert value<=8, "Maximum Gain Level is 8"
        TT_SetCameraImagerGain(self.index, value)

    @property
    def continuous_ir(self):
        """ (int)
        """
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
        """Camera Name"""
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

    def orientation_matrix(self, int matrixIndex):
        """Orientation
        ## according to TT_CameraModel() the orientation matrix
        is a 3x3 matrix."""
        return TT_CameraOrientationMatrix(self.index, matrixIndex)

    def model(self, float x, float y, float z,  #Camera Position
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
        if not TT_CameraModel(self.index, x, y, z, orientationp, principleX, principleY,
                              focalLengthX, focalLengthY, kc1, kc2, kc3, tangential0, tangential1):
            raise Exception("Could Not Set Parameters")

    @property
    @check_cam_setting
    def video_type(self):
        dict={0:"Segment Mode", 1:"Grayscale Mode", 2:"Object Mode", 3:"Precision Mode", 4:"MJPEG Mode"}
        return dict[TT_CameraVideoType(self.index)]

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

    ##@property
    ##@check_cam_setting
    ##def imager_gain_levels(self):
    ##    return  TT_CameraImagerGainLevels(self.index)  ##for some reason function always returns 0

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

    #CAMERA MASKING
    def mask(self, buffername, int bufferSize):
        assert isinstance(buffername,str), "Buffername Needs To Be String"
        cdef unsigned char * buffer=buffername
        return TT_CameraMask(self.index, buffer, bufferSize)

    def set_mask(self, buffername, int bufferSize):
        assert isinstance(buffername,str), "Buffername Needs To Be String"
        cdef unsigned char * buffer=buffername
        if not TT_SetCameraMask(self.index, buffer, bufferSize):
            raise Exception("Could Not Set Mask")

    def mask_info(self):
        cdef int blockingMaskWidth=0
        cdef int blockingMaskHeight=0
        cdef int blockingMaskGrid=0
        if TT_CameraMaskInfo(self.index, blockingMaskWidth, blockingMaskHeight, blockingMaskGrid):
            return "Camera {0} blocking masks width:{1}, height:{2}, grid:{3}".format(self.index, blockingMaskWidth, blockingMaskHeight, blockingMaskGrid)
        else:
            raise Exception("Possibly Camera {0} Has No Mask".format(self.index))

    def clear_mask(self):
        if not TT_ClearCameraMask(self.index):
            raise Exception("Could Not Clear Mask")

    #CAMERA DISTORTION
    def undistort_2d_point(self, float x, float y):
        """The 2D centroids the camera reports are distorted by the lens.  To remove the distortion, call
        CameraUndistort2DPoint.  Also if you have a 2D undistorted point that you'd like to convert back
        to a distorted point call CameraDistort2DPoint."""
        TT_CameraUndistort2DPoint(self.index, x, y)

    def distort_2d_point(self, float x, float y):
        """The 2D centroids the camera reports are distorted by the lens.  To remove the distortion, call
        CameraUndistort2DPoint.  Also if you have a 2D undistorted point that you'd like to convert back
        to a distorted point call CameraDistort2DPoint."""
        TT_CameraDistort2DPoint(self.index, x, y)

    def marker(self, int markerIndex):
        """CameraMarker fetches the 2D centroid location of the marker as seen by the camera"""
        cdef float x=0
        cdef float y=0
        if TT_CameraMarker(self.index, markerIndex, x, y):
            return "The 2D location of marker {0} is x={1}, y={2}".format(markerIndex, x, y)
        else:
            raise Exception("Could Not Fetch Location. Possibly Marker {0} Is Not Seen By Camera".format(markerIndex))

    def pixel_resolution(self):
        cdef int width=0
        cdef int height=0
        if TT_CameraPixelResolution(self.index, width, height):
            return "Pixel resolution for camera {0} is width={1}, height={2}".format(self.index, width, height)
        else:
            raise Exception

    def backproject(self, float x, float y, float z):
        """Back-project from 3D space to 2D space.  If you give this function a 3D location and select a camera,
        it will return where the point would land on the imager of that camera in to 2D space.
        This basically locates where in the camera's FOV a 3D point would be located.
        """
        cdef float cameraX=0
        cdef float cameraY=0
        TT_CameraBackproject(self.index, x, y, z, cameraX, cameraY)
        return "Point in camera 2D space: x={0}, y={1}".format(cameraX, cameraY)

    def ray(self, float x, float y):
        """Takes an undistorted 2D centroid and return a camera ray in the world coordinate system."""
        cdef float rayStartX=0
        cdef float rayStartY=0
        cdef float rayStartZ=0
        cdef float rayEndX=0
        cdef float rayEndY=0
        cdef float rayEndZ=0
        if TT_CameraRay(self.index, x, y, rayStartX, rayStartY, rayStartZ, rayEndX, rayEndY, rayEndZ):
            return "Ray Xstart={0}, Xend={1}, Ystart={2}, Yend={3}, Zstart={4}, Zend={5}".format(rayStartX, rayStartY, rayStartZ, rayEndX, rayEndY, rayEndZ)
        else:
            raise Exception

    def frame_centroid(self, int markerIndex):
        """Returns true if the camera is contributing to this 3D marker.
        It also returns the location of the 2D centroid that is reconstructing to this 3D marker"""
        cdef float x=0
        cdef float y=0
        if TT_FrameCameraCentroid(markerIndex,self.index, x, y):
            return "Camera {0} sees 2D x-position={1}, y-position={2}".format(self.index, x, y)
        else:
            raise Exception("Camera Not Contributing To 3D Position Of Marker {0}. \n Try Different Camera.".format(markerIndex))

    def frame_buffer(self, int bufferPixelWidth, int bufferPixelHeight,
                     int bufferByteSpan, int bufferPixelBitDepth, buffername):
        """Fetch the camera's frame buffer.
        This function fills the provided buffer with an image of what is in the camera view.
        The resulting image depends on what video mode the camera is in.
        If the camera is in grayscale mode, for example, a grayscale image is returned from this call."""
        assert isinstance(buffername,str), "Buffername Needs To Be String"
        cdef unsigned char * buffer=buffername
        if not TT_CameraFrameBuffer(self.index, bufferPixelWidth, bufferPixelHeight, bufferByteSpan, bufferPixelBitDepth, buffer):
            raise BufferError("Camera Frame Could Not Be Buffered")

    def frame_buffer_save_as_bmp(self, str filename):
        """Save camera's frame buffer as a BMP image file"""
        if not TT_CameraFrameBufferSaveAsBMP(self.index, filename):
            raise IOError("Camera Frame Buffer Not Successfully Saved To Filename: {0}.".format(filename))

#Functions To Set Camera Property Value, But W\O Possibility To Get Value
    def set_filter_switch(self, bool enableIRFilter):
        """True: IRFilter, False: VisibleLight"""
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



#ADDITIONAL FUNCTIONALITY
def set_frame_id_based_timing(bool enable):
    return TT_SetFrameIDBasedTiming(enable)

def set_suppress_out_of_order(bool enable):
    return TT_SetSuppressOutOfOrder(enable)

@check_npresult
def orient_tracking_bar(float positionX, float positionY, float positionZ,
                        float orientationX, float orientationY, float orientationZ, float orientationW):
    return TT_OrientTrackingBar(positionX, positionY, positionZ,
                                orientationX, orientationY, orientationZ, orientationW)

def software_build():
    """Software Release Build"""
    return TT_BuildNumber()
