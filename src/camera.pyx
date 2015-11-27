include "cnative.pxd"

from motive import utils
cimport numpy as np
import numpy as np
import warnings

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

        #Detect video modes and set as instance constants.
        self._video_modes = {'OBJECT_MODE': 0, 'GRAYSCALE_MODE': 1, 'SEGMENT_MODE': 2, 'MJPEG_MODE': 6}
        if '13' in self.name:
            self._video_modes['PRECISION_MODE'] = 4
            self.PRECISION_MODE = self._video_modes['PRECISION_MODE']

        self.OBJECT_MODE = self._video_modes['OBJECT_MODE']
        self.GRAYSCALE_MODE = self._video_modes['GRAYSCALE_MODE']
        self.SEGMENT_MODE = self._video_modes['SEGMENT_MODE']
        self.MJPEG_MODE = self._video_modes['MJPEG_MODE']


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
    @utils.decorators.check_cam_setting
    def video_mode(self):
        """0:"Segment Mode"\n 1:"Grayscale Mode"\n 2:"Object Mode"\n 4:"Precision Mode"\n 6:"MJPEG Mode" """
        return TT_CameraVideoType(self.index)

    @video_mode.setter
    def video_mode(self, value):
        assert value in self._video_modes.values(), "Possible Video Modes {0}".format(self._video_modes)
        TT_SetCameraSettings(self.index, value, self.exposure, self.threshold, self.intensity)

    @property
    @utils.decorators.check_cam_setting
    def exposure(self):
        """Camera exposure level"""
        return TT_CameraExposure(self.index)

    @exposure.setter
    def exposure(self, value):
        TT_SetCameraSettings(self.index, self.video_mode, value, self.threshold, self.intensity)

    @property
    @utils.decorators.check_cam_setting
    def threshold(self):
        """Camera threshold level for determining whether a pixel is bright enough to contain a reflective marker"""
        return TT_CameraThreshold(self.index)

    @threshold.setter
    def threshold(self, value):
        assert value >= 0 and value <= 255, "Threshold Must Be In (0,255)"
        TT_SetCameraSettings(self.index, self.video_mode, self.exposure, value, self.intensity)

    @property
    @utils.decorators.check_cam_setting
    def intensity(self):
        """Camera IR LED Brightness Intensity Level"""
        return TT_CameraIntensity(self.index)

    @intensity.setter
    def intensity(self, value):
        assert value >= 0 and value <= 15, "Intensity Must Be In (0,15)"
        TT_SetCameraSettings(self.index, self.video_mode, self.exposure, self.threshold, value)

    def set_settings(self, int video_mode, int exposure, int threshold, int intensity):
        """
        Set camera settings.  This function allows you to set the camera's video mode, exposure, threshold,
        and illumination settings.
        VideoMode: 0 = Segment Mode, 1 = Grayscale Mode, 2 = Object Mode, 4 = Precision Mode, 6 = MJPEG Mode.
        Exposure: Valid values are:  1-480.
        Threshold: Valid values are: 0-255.
        Intensity: Valid values are: 0-15  (This should be set to 15 for most situations)
        """
        return TT_SetCameraSettings(self.index, video_mode, exposure, threshold, intensity)

    @property
    @utils.decorators.check_cam_setting
    def frame_rate(self):
        """frames/sec"""
        return TT_CameraFrameRate(self.index)

    @frame_rate.setter
    def frame_rate(self, value):
        if not TT_SetCameraFrameRate(self.index, value):
            raise Exception("Could Not Set Frame Rate. Check Camera Index And Initialize With TT_Initialize()")

    @property
    @utils.decorators.check_cam_setting
    def grayscale_decimation(self):
        """returns int"""
        return  TT_CameraGrayscaleDecimation(self.index)

    @grayscale_decimation.setter
    def grayscale_decimation(self, value):
        if not TT_SetCameraGrayscaleDecimation(self.index, value):
            raise Exception("Could Not Set Decimation")

    @property
    @utils.decorators.check_cam_setting
    def image_gain(self):
        """returns int"""
        return  TT_CameraImagerGainLevels(self.index)+1   #In the motive GUI values range from 1 to 8

    @image_gain.setter
    def image_gain(self, value):
        assert 0<value<9, "Range from 1 to 8"
        TT_SetCameraImagerGain(self.index, value-1)

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
    @utils.decorators.check_cam_setting
    def id(self):
        return TT_CameraID(self.index)

    @property
    def pixel_resolution(self):
        """returns a tuple containing number of pixels in width and height per image (can depend on video_mode)"""
        cdef int width=0, height=0
        if TT_CameraPixelResolution(self.index, width, height):
            return (width, height)
        else:
            raise Exception("Could Not Find Camera Resolution")

    @property
    def frame_resolution(self):
        """depending on video mode returns different image frame resolution"""
        width, height=self.pixel_resolution
        return (width, height) if self.video_mode!=6 else (width/2, height/2)

    @property
    def location(self):
        return TT_CameraXLocation(self.index), TT_CameraYLocation(self.index), TT_CameraZLocation(self.index)

    @property
    @utils.decorators.check_cam_setting
    def max_image_gain(self):
        return  TT_CameraImagerGain(self.index)

    @property
    def is_continuous_ir_available(self):
        return TT_IsContinuousIRAvailable(self.index)

    @property
    @utils.decorators.check_cam_setting
    def temperature(self):
        return TT_CameraTemperature(self.index)

    @property
    @utils.decorators.check_cam_setting
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

    def get_frame_buffer(self):
        """
        Fetch the camera's frame buffer.
        This function fills the provided buffer with an image of what is in the camera view.
        The resulting image depends on what video mode the camera is in.
        If the camera is in grayscale mode, for example, a grayscale image is returned from this call.
        """
        # width, height=self.pixel_resolution
        # cdef unsigned char * buffer=<unsigned char *> malloc(width*height*sizeof(unsigned char))
        # if TT_CameraFrameBuffer(self.index, width, height, width, 8, buffer):
        #     py_buffer=buffer[:width*height]
        #     free(buffer)
        #     return np.frombuffer(py_buffer, dtype='B').reshape(height,width)
        #
        # else:
        #     raise BufferError("Camera Frame Could Not Be Buffered")

        width, height=self.frame_resolution
        cdef np.ndarray[unsigned char, ndim=2] frame = np.empty((height, width), dtype='B')    #np.empty is not empty due to dtype='B'
        if not TT_CameraFrameBuffer(self.index, width, height, width, 8, &frame[0,0]):
                                  #(camera number, width in pixels, height in pixels, width in bytes, bits per pixel, buffer name)
                                  #  -> where width in bytes should equal width in pixels * bits per pixel / 8
            raise BufferError("Camera Frame Could Not Be Buffered")

        # When video mode is changed, it often takes a few frames before the data coming in is correct.
        # Give a warning if you can detect that happening.
        # TODO: Get instant video mode switching.
        if len(frame) != self.frame_resolution[1]:
            print("Frame size not correct for current video mode. Call motive.update().")
            # warnings.warn("Frame size not correct for current video mode. Call motive.update().")

        unique_values = np.unique(frame)
        if self.video_mode in [self.OBJECT_MODE, self.SEGMENT_MODE]:
            if len(unique_values) > 2:
                print("Frame contains video or precision data. Call motive.update().")
                # warnings.warn("Frame contains video or precision data. Call motive.update().")
        else:
            if len(unique_values) == 2:
                print("Frame contains object or segment data. Call motive.update().")
                # warnings.warn("Frame contains object or segment data. Call motive.update().")




        return frame

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

    def set_mjpeg_high_quality(self, int mjpegQuality):                  #set to 8 minimum quality. set to 100 maximum quality. range still has to be checked
        if not TT_SetCameraMJPEGHighQuality(self.index, mjpegQuality):
            raise Exception("Could Not Enable HighQuality. Possibly Camera Has No HighQuality For MJPEG")