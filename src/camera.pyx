include "cnative.pxd"

# Decorator
def check_cam_setting(func):
    """  Decorator to check if calling a TT_Camera function returns an exception. """
    def wrapper(*args, **kwargs):
        check=func(*args, **kwargs)
        if check<0:
            raise Exception("Value Not Available. Usually Camera Index Not Valid Or Devices Not Initialized")
        else:
            return check
    return wrapper


def get_cams():
    """Initiate all cameras as python objects, where camera #k is cam[k-1]"""
    return tuple(Camera(cameraIndex) for cameraIndex in xrange(TT_CameraCount()))

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


#CLASS
class Camera(object):

    def __init__(self, cameraIndex):
        assert cameraIndex < TT_CameraCount(), "There Are Only {0} Cameras".format(TT_CameraCount())
        self.index=cameraIndex

    def __str__(self):
        return "Camera Object {0}: {1}".format(self.index, TT_CameraName(self.index))

    def __repr__(self):
        return self.__str__()

    @property
    def name(self):
        """Camera Name"""
        return TT_CameraName(self.index)

    @property
    def group(self):
        """Camera's camera group index"""
        return TT_CamerasGroup(self.index)

    @group.setter
    def group(self,value):
        TT_SetCameraGroup(self.index, value)

    @property
    @check_cam_setting
    def video_type(self):
        """0:"Segment Mode"\n 1:"Grayscale Mode"\n 2:"Object Mode"\n 3:"Precision Mode"\n 4:"MJPEG Mode" """
        return TT_CameraVideoType(self.index)

    @video_type.setter
    def video_type(self, value):
        assert value in {0:"Segment Mode", 1:"Grayscale Mode", 2:"Object Mode", 3:"Precision Mode", 4:"MJPEG Mode"}, "Video Type Must Be In (0,4)"
        TT_SetCameraSettings(self.index, value, self.exposure, self.threshold, self.intensity)

    @property
    @check_cam_setting
    def exposure(self):
        """Camera exposure level"""
        return TT_CameraExposure(self.index)

    @exposure.setter
    def exposure(self, value):
        TT_SetCameraSettings(self.index, self.video_type, value, self.threshold, self.intensity)

    @property
    @check_cam_setting
    def threshold(self):
        """Camera threshold level for determining whether a pixel is bright enough to contain a reflective marker"""
        return TT_CameraThreshold(self.index)

    @threshold.setter
    def threshold(self, value):
        assert value >= 0 and value <= 255, "Threshold Must Be In (0,255)"
        TT_SetCameraSettings(self.index, self.video_type, self.exposure, value, self.intensity)

    @property
    @check_cam_setting
    def intensity(self):
        """Camera IR LED Brightness Intensity Level"""
        return TT_CameraIntensity(self.index)

    @intensity.setter
    def intensity(self, value):
        assert value >= 0 and value <= 15, "Intensity Must Be In (0,15)"
        TT_SetCameraSettings(self.index, self.video_type, self.exposure, self.threshold, value)

    def set_settings(self, int videotype, int exposure, int threshold, int intensity):
        """
        Set camera settings.  This function allows you to set the camera's video mode, exposure, threshold,
        and illumination settings.
        VideoType: 0 = Segment Mode, 1 = Grayscale Mode, 2 = Object Mode, 4 = Precision Mode, 6 = MJPEG Mode.
        Exposure: Valid values are:  1-480.
        Threshold: Valid values are: 0-255.
        Intensity: Valid values are: 0-15  (This should be set to 15 for most situations)
        """
        return TT_SetCameraSettings(self.index, videotype, exposure, threshold, intensity)

    @property
    @check_cam_setting
    def frame_rate(self):
        """frames/sec"""
        return TT_CameraFrameRate(self.index)

    @frame_rate.setter
    def frame_rate(self, value):
        if not TT_SetCameraFrameRate(self.index, value):
            raise Exception("Could Not Set Frame Rate. Check Camera Index And Initialize With TT_Initialize()")

    @property
    @check_cam_setting
    def grayscale_decimation(self):
        """returns int"""
        return  TT_CameraGrayscaleDecimation(self.index)

    @grayscale_decimation.setter
    def grayscale_decimation(self, value):
        if not TT_SetCameraGrayscaleDecimation(self.index, value):
            raise Exception("Could Not Set Decimation")

    @property
    @check_cam_setting
    def imager_gain(self):
        """returns int"""
        return  TT_CameraImagerGainLevels(self.index)

    @imager_gain.setter
    def imager_gain(self, value):
        assert value<=8, "Maximum Gain Level is 8"
        TT_SetCameraImagerGain(self.index, value)

    @property
    def continuous_ir(self):
        """returns bool"""
        assert TT_IsContinuousIRAvailable(self.index), "Camera {0} Does Not Support Continuous IR".format(self.index)
        return TT_ContinuousIR(self.index)

    @continuous_ir.setter
    def continuous_ir(self, bool value):
        TT_SetContinuousIR(self.index, value)

#def set_continuous_camera_mjpeg_high_quality_ir(int cameraIndex, bool Enable):
#    TT_SetContinuousTT_SetCameraMJPEGHighQualityIR(cameraIndex, Enable)
#    print "Set"

#Properties Without Simple Setter (If Not Here Maybe In Camera Class)
    @property
    @check_cam_setting
    def id(self):
        return TT_CameraID(self.index)

    @property
    def location(self):
        return TT_CameraXLocation(self.index), TT_CameraYLocation(self.index), TT_CameraZLocation(self.index)

    def orientation_matrix(self, int matrixIndex):
        """According to TT_CameraModel() the orientation matrix is a 3x3 matrix"""
        return TT_CameraOrientationMatrix(self.index, matrixIndex)

    def model(self, float x, float y, float z,             #Camera Position
              orientation,                                 #Orientation (3x3 matrix)
              float principleX, float principleY,          #Lens center (in pixels)
              float focalLengthX, float focalLengthY,      #Lens focal  (in pixels)
              float kc1, float kc2, float kc3,             #Barrel distortion coefficients
              float tangential0, float tangential1):       #Tangential distortion
        """
        Set a camera's extrinsic (position & orientation) and intrinsic (lens distortion) parameters
        with parameters compatible with the OpenCV intrinsic model.
        """
        cdef float orientationp[9]
        for i in range(0,9):
            orientationp[i]=orientation[i]
        if not TT_CameraModel(self.index, x, y, z, orientationp, principleX, principleY,
                              focalLengthX, focalLengthY, kc1, kc2, kc3, tangential0, tangential1):
            raise Exception("Could Not Set Parameters")

    @property
    @check_cam_setting
    def max_imager_gain(self):
        return  TT_CameraImagerGain(self.index)

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

    @property
    def markers(self):
        """A tuple of 2D centroid locations of the marker as seen by the camera"""
        cdef float x = 0, y = 0
        markers = []
        for markerIndex in xrange(TT_CameraMarkerCount(self.index)):
            TT_CameraMarker(self.index, markerIndex, x, y)
            markers.append((x, y))
        return tuple(markers)


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
        cdef int blockingMaskWidth=0, blockingMaskHeight=0, blockingMaskGrid=0
        if TT_CameraMaskInfo(self.index, blockingMaskWidth, blockingMaskHeight, blockingMaskGrid):
            return {'blockingMaskwidth':blockingMaskWidth,'blockingMaskHeight': blockingMaskHeight, 'blockingMaskGrid': blockingMaskGrid}
        else:
            raise Exception("Possibly Camera {0} Has No Mask".format(self.index))

    def clear_mask(self):
        if not TT_ClearCameraMask(self.index):
            raise Exception("Could Not Clear Mask")

    #CAMERA DISTORTION
    def undistort_2d_point(self, float x, float y):
        """
        The 2D centroids the camera reports are distorted by the lens.  To remove the distortion, call
        CameraUndistort2DPoint.  Also if you have a 2D undistorted point that you'd like to convert back
        to a distorted point call CameraDistort2DPoint.
        """
        TT_CameraUndistort2DPoint(self.index, x, y)

    def distort_2d_point(self, float x, float y):
        """
        The 2D centroids the camera reports are distorted by the lens.  To remove the distortion, call
        CameraUndistort2DPoint.  Also if you have a 2D undistorted point that you'd like to convert back
        to a distorted point call CameraDistort2DPoint.
        """
        TT_CameraDistort2DPoint(self.index, x, y)

    def pixel_resolution(self):
        cdef int width=0, height=0
        if TT_CameraPixelResolution(self.index, width, height):
            return {'width':width, 'height':height}
        else:
            raise Exception

    def backproject(self, float x, float y, float z):
        """
        Back-project from 3D space to 2D space.  If you give this function a 3D location and select a camera,
        it will return where the point would land on the imager of that camera in to 2D space.
        This basically locates where in the camera's FOV a 3D point would be located.
        """
        cdef float cameraX=0, cameraY=0
        TT_CameraBackproject(self.index, x, y, z, cameraX, cameraY)
        return cameraX, cameraY

    def ray(self, float x, float y):
        """Takes an undistorted 2D centroid and returns a camera ray in the world coordinate system"""
        cdef float rayStartX=0, rayStartY=0, rayStartZ=0, rayEndX=0, rayEndY=0, rayEndZ=0
        if TT_CameraRay(self.index, x, y, rayStartX, rayStartY, rayStartZ, rayEndX, rayEndY, rayEndZ):
            return {'Xstart':rayStartX, 'Ystart':rayStartY, 'Zstart':rayStartZ,'Xend':rayEndX, 'Yend':rayEndY, 'Zend':rayEndZ}
        else:
            raise Exception

    def frame_centroid(self, int markerIndex):
        """
        Returns true if the camera is contributing to this 3D marker.
        It also returns the location of the 2D centroid that is reconstructing to this 3D marker
        """
        cdef float x=0, y=0
        if TT_FrameCameraCentroid(markerIndex,self.index, x, y):
            return x, y
        else:
            raise Exception("Camera Not Contributing To 3D Position Of Marker {0}. \n Try Different Camera.".format(markerIndex))

    def frame_buffer(self, int bufferPixelWidth, int bufferPixelHeight,
                     int bufferByteSpan, int bufferPixelBitDepth, buffername):
        """
        Fetch the camera's frame buffer.
        This function fills the provided buffer with an image of what is in the camera view.
        The resulting image depends on what video mode the camera is in.
        If the camera is in grayscale mode, for example, a grayscale image is returned from this call.
        """
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