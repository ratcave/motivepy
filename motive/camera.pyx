"""Motive Camera Module

This module features the functionality to communicate with specific
cameras. It is basically made up of one large camera class.
Once a camera has been initialized and called as a python object,
various methods to change its settings and collect data from it,
can be applied.
The basic function in this module is get_cams().

Examples::

    >>>get_cams()
    (Camera Object 0: Camera Prime 13 #11000, Camera Object 1: Camera Prime 17W #10187,
    Camera Object 2: Camera Prime 17W #10189)
    >>>cams=get_cams()
    >>>cams[1].name
    Camera Prime 17W #10187

"""
from __future__ import absolute_import

include "cnative.pxd"
cdef extern from *:
    wchar_t* PyUnicode_AsWideCharString(object, Py_ssize_t *size)
    object PyUnicode_FromWideChar(const wchar_t *w, Py_ssize_t size)

from .decorators import convert_string_output
cimport numpy as np
import numpy as np
from collections import namedtuple

Mask = namedtuple('Mask', 'grid width height')
Resolution = namedtuple('Resolution', 'width height')
CameraSettings = namedtuple('CameraSettings', 'video_mode exposure threshold intensity')


def check_cam_setting(func):
    """Decorator that checks if a Camera function can be called correctly

    Returns:
        check(int): An integer value encoding the camera setting (see camera.pyx)
    Raises:
        Exception: If the camera function returns a value encoding an error
    """
    def wrapper(*args, **kwargs):
        check=func(*args, **kwargs)
        if check < 0:
            raise Exception("Value Not Available. Usually Camera Index Not Valid Or Devices Not Initialized")
        else:
            return check
    return wrapper


def get_cams():
    """Returns a tuple containing all cameras."""
    return Camera.get_all()


class Camera(object):

    OBJECT_MODE = kVideoType_Object
    GRAYSCALE_MODE = kVideoType_Grayscale
    SEGMENT_MODE = kVideoType_Segment
    PRECISION_MODE = kVideoType_Precision
    MJPEG_MODE = kVideoType_MJPEG

    def __init__(self, index):
        """Returns a Camera object."""

        if index < 0:
            raise ValueError("Camera Index Must be Positive")
        if index >= self.count():
            raise ValueError("Index too high -- There Are Only {0} Cameras").format(self.count())

        self.index=index

    @staticmethod
    def count():
        return CameraCount()

    @classmethod
    def get_all(cls):
        """Returns tuple of Cameras"""
        return tuple(cls(index) for index in range(cls.count()))

    def __str__(self):
        return '{cls}(index={idx}, name="{name}")'.format(cls=self.__class__.__name__, idx=self.index, name=self.name)

    def __repr__(self):
        return self.__str__()

    @property
    def name(self):
        """str: Camera name"""
        cdef wchar_t name[256]
        CameraName(self.index, name, 256)
        return PyUnicode_FromWideChar(name, -1)

    @property
    def id(self):
        "int: Camera ID number"
        return check_cam_setting(CameraID)(self.index)

    @property
    def group(self):
        """int: Camera's group index"""
        return CameraGroup(self.index)

    @property
    def enabled(self):
        """boole: whether the camera is enabled"""
        cdef eCameraState currentState
        CameraState(self.index, currentState)
        return currentState == Camera_Enabled

    @enabled.setter
    def enabled(self, value):
        if value:
            SetCameraState(self.index, Camera_Enabled)
        else:
            SetCameraState(self.index, Camera_Disabled)

    @property
    def video_mode(self):
        """int: Integer encoding the actual video mode of the camera. See Camera.X_MODE"""
        return check_cam_setting(CameraVideoType)(self.index)

    @video_mode.setter
    def video_mode(self, value):
        self.set_settings(video_mode=value, exposure=self.exposure, threshold=self.threshold, intensity=self.intensity)

    @property
    def exposure(self):
        """int: Camera exposure level"""
        return check_cam_setting(CameraExposure)(self.index)

    @exposure.setter
    def exposure(self, value):
        self.set_settings(video_mode=self.video_mode, exposure=value, threshold=self.threshold, intensity=self.intensity)
        SetCameraSettings(self.index, self.video_mode, value, self.threshold, self.intensity)

    @property
    def threshold(self):
        """int: Camera threshold level for determining whether a pixel is bright enough to contain a reflective marker"""
        return check_cam_setting(CameraThreshold)(self.index)

    @threshold.setter
    def threshold(self, value):
        self.set_settings(video_mode=self.video_mode, exposure=self.exposure, threshold=value, intensity=self.intensity)

    @property
    def settings(self):
        return CameraSettings(video_mode=self.video_mode, exposure=self.exposure, threshold=self.threshold, intensity=self.intensity)

    @settings.setter
    def settings(self, value):
        """Takes a CameraSettings tuple (video_mode, exposure, threshold, intensity) to apply multiple settings at once."""
        ss = CameraSettings(*value) if not isinstance(value, CameraSettings) else value

        if ss.video_mode == self.PRECISION_MODE and '13' not in self.name.split('#')[0]:
            raise ValueError("video_mode PRECISION_MODE not available for this Camera.")

        if not (0 <= ss.intensity <= 15):
            raise ValueError("Intensity Must Be In (0,15)")

        if not (0 <= ss.threshold <= 255):
            raise ValueError("Threshold Must Be In (0,255)")

        return SetCameraSettings(self.index, ss.video_mode, ss.exposure, ss.threshold, ss.intensity)


    def set_settings(self, int video_mode, int exposure, int threshold, int intensity):
        """Set camera settings

        This function allows you to set the camera's video mode, exposure, threshold,
        and illumination settings.

        Args:
            video_mode (int): Integer encoding the actual video mode of the camera. See Camera.X_MODE
            exposure (int): Camera exposure level
            threshold (int): Camera threshold level for determining whether a pixel is bright enough to contain a reflective marker
            intensity (int): Camera IR LED brightness intensity level
        """
        # raise DeprecationWarning("Camera.set_settings() deprecated.  New use is property Camera.settings.")

        if video_mode == self.PRECISION_MODE and '13' not in self.name.split('#')[0]:
            raise ValueError("video_mode PRECISION_MODE not available for this Camera.")

        if not (0 <= intensity <= 15):
            raise ValueError("Intensity Must Be In (0,15)")

        if not (0 <= threshold <= 255):
            raise ValueError("Threshold Must Be In (0,255)")

        return SetCameraSettings(self.index, video_mode, exposure, threshold, intensity)

    @property
    def grayscale_decimation(self):
        """int: level of decimation of frame capture (how many frames to skip when getting video)"""
        raise NotImplementedError
        return  check_cam_setting(CameraGrayscaleDecimation)(self.index)

    @grayscale_decimation.setter
    def grayscale_decimation(self, value):
        raise NotImplementedError
        if not SetCameraGrayscaleDecimation(self.index, value):
            raise Exception("Could Not Set Decimation")

    @property
    def image_gain(self):
        """int: Camera gain level

        Raises:
            AssertionError: If setting values out of scope
        """
        return  check_cam_setting(CameraImagerGainLevels)(self.index)+1   #In the motive GUI values range from 1 to 8

    @image_gain.setter
    def image_gain(self, value):
        if not 0 <= value <= 8:
            raise ValueError("Camera.image_gain must be between 0 and 8")
        SetCameraImagerGain(self.index, value-1)

    @property
    def ir_led_on(self):
        """bool: IR illumination is on"""
        return CameraIRLedsOn(self.index)

    @ir_led_on.setter
    def ir_led_on(self, bool value):
        SetCameraIRLedsOn(self.index, value)

    @property
    def pixel_resolution(self):
        """Tuple[int]: Number of pixels in width and height for this camera

        Note:
            Resolution of actual frame can depend on video mode
        Raises:
            Exception: If the resolution can not be found
        """
        cdef int width = 0, height = 0
        if CameraPixelResolution(self.index, width, height):
            return Resolution(width=width, height=height)
        else:
            raise Exception("Could Not Find Camera Resolution")

    @property
    def frame_resolution(self):
        """Tuple[int]: Number of pixels in width and height for this frame"""
        width, height = self.pixel_resolution
        if self.video_mode == self.MJPEG_MODE:
            width, height = width / 2, height / 2
        return Resolution(width=width, height=height)

    @property
    def max_image_gain(self):
        """int: Maximum possible gain level of camera"""
        return  check_cam_setting(CameraImagerGain)(self.index)

    @property
    def temperature(self):
        """float: Temperature of camera"""
        return check_cam_setting(CameraTemperature)(self.index)

    @property
    def ring_light_temperature(self):
        """float: Temperature of the cameras LED ring"""
        return  check_cam_setting(CameraRinglightTemperature)(self.index)

    @property
    def marker_count(self):
        """int: Number of markers as seen by camera"""
        return CameraMarkerCount(self.index)

    @property
    def markers(self):
        """Tuple[float]: 2D centroid locations of all markers as seen by the camera"""
        cdef float x = 0., y = 0.
        markers = []
        for markerIndex in xrange(CameraMarkerCount(self.index)):
            CameraMarker(self.index, markerIndex, x, y)
            markers.append((x, y))
        return tuple(markers)

    @property
    def location(self):
        """Tuple[float]: 3D camera location"""
        return CameraXLocation(self.index), CameraYLocation(self.index), CameraZLocation(self.index)

    @property
    def orientation_matrix(self):
        """Returns the Camera's 3x3 orientation matrix."""
        ori_mat = np.zeros(shape=(3, 3), dtype=float)
        for idx in range(9):
            ori_mat[np.unravel_index(idx, ori_mat.shape)] = CameraOrientationMatrix(self.index, idx)
        return ori_mat

    def model(self, float x, float y, float z,
              orientation,
              float principleX, float principleY,
              float focalLengthX, float focalLengthY,
              float kc1, float kc2, float kc3,
              float tangential0, float tangential1):
        """Sets a camera's extrinsic (position & orientation) and intrinsic (lens distortion) parameters

        Those parameters are compatible with the OpenCV intrinsic model.

        Args:
            x(float): Cameras X position
            y(float): Cameras Y position
            z(float): Cameras Z position
            orientation(List[float]): The 3x3 camera orientation matrix
            principleX(float): Cameras lens center X position in pixels
            principleY(float): Cameras lens center Y position in pixels
            focalLengthX(float): Cameras lens focal length in X direction in pixels
            focalLengthY(float): Cameras lens focal length in Y direction in pixels
            kc1(float): Cameras first barrel distortion coefficient
            kc2(float): Cameras second barrel distortion coefficient
            kc3(float): Cameras third barrel distortion coefficient
            tangential0(float): Cameras first tangential distortion coefficient
            tangential1(float): Cameras' second tangential distortion coefficient

         Raises:
            Exception: If the parameters could not be set
        """
        cdef float orientationp[9]
        for i in range(0,9):
            orientationp[i]=orientation[i]
        if not CameraModel(self.index, x, y, z, orientationp, principleX, principleY,
                              focalLengthX, focalLengthY, kc1, kc2, kc3, tangential0, tangential1):
            raise Exception("Could Not Set Parameters")

    #TODO: Create camera mask class. See issue list on github
    #CAMERA MASKING
    def mask(self, buffer, int bufferSize):
        raise NotImplementedError
        cdef unsigned char * buffer=buffer          #buffer should be an integer array. See get_frame_buffer() below for example
        return CameraMask(self.index, buffer, bufferSize)

    def set_mask(self, buffer, int bufferSize):
        raise NotImplementedError
        cdef unsigned char * buffer = buffer
        if not SetCameraMask(self.index, buffer, bufferSize):
            raise Exception("Could Not Set Mask")

    @property
    def mask_info(self):
        cdef int blockingMaskWidth=0, blockingMaskHeight=0, blockingMaskGrid=0
        if CameraMaskInfo(self.index, blockingMaskWidth, blockingMaskHeight, blockingMaskGrid):
            return Mask(width=blockingMaskWidth, height=blockingMaskHeight, grid=blockingMaskGrid)
        else:
            raise Exception("Possibly Camera {0} Has No Mask".format(self.index))

    def clear_mask(self):
        if not ClearCameraMask(self.index):
            raise Exception("Could Not Clear Mask")

    #CAMERA DISTORTION
    def undistort_2d_point(self, float x, float y):
        """Returns undistorted 2D point coordinates

        The 2D centroids the camera reports are distorted by the lens. This function
        removes the distortion.

        Args:
            x(float): Distorted X coordinate
            y(float): Distorted Y coordinate
        Returns:
            Tuple[float]: The undistorted point coordinates
        """
        CameraUndistort2DPoint(self.index, x, y)
        return x,y

    def distort_2d_point(self, float x, float y):
        """Returns distorted 2D point coordinates

        The 2D centroids the camera reports are distorted by the lens. If you already
        undistorted those points, this function distorts them again.

        Args:
            x(float): Undistorted X coordinate
            y(float): Undistorted Y coordinate
        Returns:
            Tuple[float]: The distorted point coordinates
        """
        CameraDistort2DPoint(self.index, x, y)
        return x,y

    def backproject(self, float x, float y, float z):
        """Back-project 3D coordinates of point to 2D camera coordinates of point

        If you give this function a 3D location and select a camera, it will return
        where the point would land on the imager of that camera in to 2D space.
        This basically locates where in the camera's FOV a 3D point would be located.

        Args:
            x(float): X coordinate of point
            y(float): Y coordinate of point
            z(float): Z coordinate of point
        Returns:
            Tuple[float]: The x,y coordinates of a 3D point as seen by the camera
        """
        cdef float cameraX=0, cameraY=0
        CameraBackproject(self.index, x, y, z, cameraX, cameraY)
        return cameraX, cameraY

    def ray(self, float x, float y):
        """Takes an undistorted 2D centroid and returns a camera ray in the world coordinate system

        Args:
            float(x): X coordinate of the centroid
            float(y): Y coordinate of the centroid
        Returns:
            Dictionary, maps identifying strings to start position coordinates and
            end position coordinates of ray
        Raises:
            Exception: If the function transforming from centroid to ray fails
        """
        cdef float x0 = 0., y0 = 0., z0 = 0., x1 = 0., y1 = 0., z1 = 0.
        if CameraRay(self.index, x, y, x0, y0, z0, x1, y1, z1):
            return ((x0, y0, z0), (x1, y1, z1))
        else:
            raise Exception

    def frame_centroid(self, int markerIndex):
        """Returns the location of the 2D centroid back-projection of a specific 3D Marker
        if the camera is contributing to this marker.

        Args:
            markerIndex(int): The index of the marker
        Returns:
            Tuple[float]: The 2D coordinates of the back-projected marker
        Raises:
            Exception: If the camera does not contribute to this marker
        """
        cdef float x=0, y=0
        if FrameCameraCentroid(markerIndex,self.index, x, y):
            return True, x, y
        else:
            return False, 0, 0
            #raise Exception("Camera Not Contributing To 3D Position Of Marker {0}. \n Try Different Camera.".format(markerIndex))

    @property
    def frame_buffer(self):
        """The Camera's frame buffer"""
        res = self.frame_resolution
        cdef np.ndarray[unsigned char, ndim=2] frame = np.empty((res.height, res.width), dtype='B')    #after this call frame is array with zeros
        if not CameraFrameBuffer(self.index, res.width, res.height, res.width, 8, &frame[0,0]):
            raise BufferError("Camera Frame Could Not Be Buffered")

        #When video mode is changed, it often takes a few frames before the data coming in is correct.
        # Give a warning if you can detect that happening.
        #TODO: Get instant video mode switching.
        #TODO: np.unique slows down the whole process considerably. Maybe only check framesize
        #TODO: find some fast way of checking most important error
        # if len(frame) != self.frame_resolution[1]:
        #     print("Frame size not correct for current video mode. Call motive.update().")
        #     # warnings.warn("Frame size not correct for current video mode. Call motive.update().")
        #
        # unique_values = np.unique(frame)
        # if self.video_mode in [self.OBJECT_MODE, self.SEGMENT_MODE]:
        #     if len(unique_values) > 2:
        #         print("Frame contains video or precision data. Call motive.update().")
        #         # warnings.warn("Frame contains video or precision data. Call motive.update().")
        # else:
        #     if len(unique_values) == 2:
        #         print("Frame contains object or segment data. Call motive.update().")
        #         # warnings.warn("Frame contains object or segment data. Call motive.update().")
        #

        return frame

#Functions To Set Camera Property Value, But W\O Possibility To Get Value
    def set_filter_switch(self, bool enableIRFilter):
        """Switch filter of camera

        Args:
            enableIRFilter(bool): True, IR filter is on. False, visible light.
        Raises:
            Exception: If camera could not switch filter
        """
        if not SetCameraFilterSwitch(self.index, enableIRFilter):
            raise Exception("Could Not Switch Filter. Possibly Camera Has No IR Filter")

    def set_agc(self, bool enableAutomaticGainControl):
        """Switch gain control

        Args:
            enableAutomaticGainControl(bool): True, AGC is on. False,  manual GC.
        Raises:
            Exception: If camera could not enable AGC
        """
        if not SetCameraAGC(self.index, enableAutomaticGainControl):
            raise Exception("Could Not Enable AGC. Possibly Camera Has No AGC")

    def set_aec(self, bool enableAutomaticExposureControl):
        """Switch exposure control

        Args:
            enableAutomaticExposureControl(bool): True, AEC is on. False,  manual EC.
        Raises:
            Exception: If camera could not enable AEC
        """
        if not SetCameraAEC(self.index, enableAutomaticExposureControl):
            raise Exception("Could Not Enable AEC. Possibly Camera Has No AEC")

    def set_high_power(self, bool enableHighPowerMode):
        """Set camera IR LED to high power mode

        Args:
            enableHighPowerMode(bool): True, HPM is on. False,  manual LED intensity with max 15.
        Raises:
            Exception: If camera could not enable HPM
        """
        if not SetCameraHighPower(self.index, enableHighPowerMode):
            raise Exception("Could Not Enable HighPowerMode. Possibly Camera Has No HighPowerMode")

    def set_mjpeg_high_quality(self, int mjpegQuality):                  #set to 8 minimum quality. set to 100 maximum quality. TODO: detailed range check
        """Set camera mjpeg video mode to higher quality

        Args:
            mjpegQuality(int): Value determines MJPEG quality
        Raises:
            Exception: If camera could not change quality
        """
        if not SetCameraMJPEGQuality(self.index, mjpegQuality):
            raise Exception("Could Not Enable HighQuality. Possibly Camera Has No HighQuality For MJPEG")