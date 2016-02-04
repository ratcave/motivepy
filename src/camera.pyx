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

include "cnative.pxd"

from motive import utils
cimport numpy as np
import numpy as np
import warnings
import cv
import cv2
import time

def get_cams():
    """Returns a tuple containing all cameras

    Returns:
        Tuple of camera objects.
    """
    return tuple(Camera(cameraIndex) for cameraIndex in xrange(TT_CameraCount()))

#CAMERA GROUP SUPPORT
def camera_group_count():
    """Returns number of camera groups"""
    return TT_CameraGroupCount()

def create_camera_group():
    """Adds an additional camera group

    Raises:
        Exception: If a new camera group could not be created
    """
    if not TT_CreateCameraGroup():
        raise Exception("Could Not Create Camera Group")

def remove_camera_group(int groupIndex):
    """Removes a camera group

    Note:
        A camera group can only be removed if it contains
        at least one camera.
    Args:
        groupIndex (int): The index of the camera group to be removed
    Raises:
        Exception: If the camera group could not be removed
    """
    if not TT_RemoveCameraGroup(groupIndex):
        raise Exception("Could Not Remove. Check If Group Empty")


def set_group_shutter_delay(int groupIndex, int microseconds):
    """Set camera group's shutter delay

    Args:
        groupIndex (int): The index of the camera group to be removed
        microseconds (int): The time between opening of shutter and capture of frame
    """
    raise NotImplementedError
    TT_SetGroupShutterDelay(groupIndex, microseconds)


#CAMERA CLASS
class Camera(object):

    def __init__(self, cameraIndex):
        """Returns a camera object

        Args:
            cameraIndex (int): The index of the camera to be returned
        Raises:
            AssertionError: If the index is larger than the number of cameras
        """
        assert cameraIndex < TT_CameraCount(), "There Are Only {0} Cameras".format(TT_CameraCount())
        self.index=cameraIndex

        #Detect video modes and set as instance constants.
        self._video_modes = {'OBJECT_MODE': 0, 'GRAYSCALE_MODE': 1, 'SEGMENT_MODE': 2, 'MJPEG_MODE': 6}
        if '13' in self.name:
            self._video_modes['PRECISION_MODE'] = 4
            self.PRECISION_MODE = self._video_modes['PRECISION_MODE']
            """int: Integer code for video in precision mode
            Note:
                Only Prime Cameras of the 13 series can switch to precision mode
            """

        self.OBJECT_MODE = self._video_modes['OBJECT_MODE']
        """int: Integer code for video in object mode"""

        self.GRAYSCALE_MODE = self._video_modes['GRAYSCALE_MODE']
        """int: Integer code for video in grayscale mode"""

        self.SEGMENT_MODE = self._video_modes['SEGMENT_MODE']
        """int: Integer code for video in segment mode"""

        self.MJPEG_MODE = self._video_modes['MJPEG_MODE']
        """int: Integer code for video in mjpeg mode"""


    def __str__(self):
        return "Camera Object {0}: {1}".format(self.index, TT_CameraName(self.index))

    def __repr__(self):
        return self.__str__()

    @property
    def name(self):
        """str: Camera name"""
        return TT_CameraName(self.index)

    @property
    def group(self):
        """int: Camera's group index"""
        return TT_CamerasGroup(self.index)

    @group.setter
    def group(self,value):
        TT_SetCameraGroup(self.index, value)

    @property
    @utils.decorators.check_cam_setting
    def video_mode(self):
        """int: Integer encoding the actual video mode of the camera. See Camera.X_MODE"""
        return TT_CameraVideoType(self.index)

    @video_mode.setter
    def video_mode(self, value):
        assert value in self._video_modes.values(), "Possible Video Modes {0}".format(self._video_modes)
        TT_SetCameraSettings(self.index, value, self.exposure, self.threshold, self.intensity)

    @property
    @utils.decorators.check_cam_setting
    def exposure(self):
        """int: Camera exposure level"""
        return TT_CameraExposure(self.index)

    @exposure.setter
    def exposure(self, value):
        TT_SetCameraSettings(self.index, self.video_mode, value, self.threshold, self.intensity)

    @property
    @utils.decorators.check_cam_setting
    def threshold(self):
        """int: Camera threshold level for determining whether a pixel is bright enough to contain a reflective marker

        Raises:
            AssertionError: If setting values out of scope
        """
        return TT_CameraThreshold(self.index)

    @threshold.setter
    def threshold(self, value):
        assert value >= 0 and value <= 255, "Threshold Must Be In (0,255)"
        TT_SetCameraSettings(self.index, self.video_mode, self.exposure, value, self.intensity)

    @property
    @utils.decorators.check_cam_setting
    def intensity(self):
        """int: Camera IR LED brightness intensity level

        Raises:
            AssertionError: If setting values out of scope
        """
        return TT_CameraIntensity(self.index)

    @intensity.setter
    def intensity(self, value):
        assert value >= 0 and value <= 15, "Intensity Must Be In (0,15)"
        TT_SetCameraSettings(self.index, self.video_mode, self.exposure, self.threshold, value)

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
        return TT_SetCameraSettings(self.index, video_mode, exposure, threshold, intensity)

    @property
    @utils.decorators.check_cam_setting
    def frame_rate(self):
        """int: Cameras frame rate in Hz. That is frames per second"""
        return TT_CameraFrameRate(self.index)

    @frame_rate.setter
    def frame_rate(self, value):
        if not TT_SetCameraFrameRate(self.index, value):
            raise Exception("Could Not Set Frame Rate. Check Camera Index And Initialize With TT_Initialize()")

    @property
    @utils.decorators.check_cam_setting
    def grayscale_decimation(self):
        """int: level of decimation of frame capture"""
        raise NotImplementedError
        return  TT_CameraGrayscaleDecimation(self.index)

    @grayscale_decimation.setter
    def grayscale_decimation(self, value):
        raise NotImplementedError
        if not TT_SetCameraGrayscaleDecimation(self.index, value):
            raise Exception("Could Not Set Decimation")

    @property
    @utils.decorators.check_cam_setting
    def image_gain(self):
        """int: Camera gain level

        Raises:
            AssertionError: If setting values out of scope
        """
        return  TT_CameraImagerGainLevels(self.index)+1   #In the motive GUI values range from 1 to 8

    @image_gain.setter
    def image_gain(self, value):
        assert 0<value<9, "Range from 1 to 8"
        TT_SetCameraImagerGain(self.index, value-1)

    @property
    def continuous_ir(self):
        """bool: Continuous IR illumination is on"""
        assert TT_IsContinuousIRAvailable(self.index), "Camera {0} Does Not Support Continuous IR".format(self.index)
        return TT_ContinuousIR(self.index)

    @continuous_ir.setter
    def continuous_ir(self, bool value):
        TT_SetContinuousIR(self.index, value)

def set_continuous_camera_mjpeg_high_quality_ir(int cameraIndex, bool Enable):
    raise NotImplementedError
    TT_SetContinuousTT_SetCameraMJPEGHighQualityIR(cameraIndex, Enable)
    print "Set"

#Properties Without Simple Setter (If Not Here Maybe In Camera Class of CameraSDK)
    @property
    @utils.decorators.check_cam_setting
    def id(self):
        "int: Camera ID number"
        return TT_CameraID(self.index)

    @property
    def pixel_resolution(self):
        """Tuple[int]: Number of pixels in width and height for this camera

        Note:
            Resolution of actual frame can depend on video mode
        Raises:
            Exception: If the resolution can not be found
        """
        cdef int width=0, height=0
        if TT_CameraPixelResolution(self.index, width, height):
            return (width, height)
        else:
            raise Exception("Could Not Find Camera Resolution")

    @property
    def frame_resolution(self):
        """Tuple[int]: Number of pixels in width and height for this frame"""
        width, height=self.pixel_resolution
        return (width, height) if self.video_mode!=6 else (width/2, height/2)

    @property
    @utils.decorators.check_cam_setting
    def max_image_gain(self):
        """int: Maximum possible gain level of camera"""
        return  TT_CameraImagerGain(self.index)

    @property
    def is_continuous_ir_available(self):
        """bool: Continuous IR illumination is available"""
        return TT_IsContinuousIRAvailable(self.index)

    @property
    @utils.decorators.check_cam_setting
    def temperature(self):
        """float: Temperature of camera"""
        return TT_CameraTemperature(self.index)

    @property
    @utils.decorators.check_cam_setting
    def ring_light_temperature(self):
        """float: Temperature of the cameras LED ring"""
        return  TT_CameraRinglightTemperature(self.index)

    @property
    def marker_count(self):
        """int: Number of markers as seen by camera"""
        return TT_CameraMarkerCount(self.index)

    @property
    def markers(self):
        """Tuple[float]: 2D centroid locations of all markers as seen by the camera"""
        cdef float x = 0, y = 0
        markers = []
        for markerIndex in xrange(TT_CameraMarkerCount(self.index)):
            TT_CameraMarker(self.index, markerIndex, x, y)
            markers.append((x, y))
        return tuple(markers)

    @property
    def location(self):
        """Tuple[float]: 3D camera location"""
        return TT_CameraXLocation(self.index), TT_CameraYLocation(self.index), TT_CameraZLocation(self.index)

    def orientation_matrix(self, int matrixIndex):
        """Returns one element of a 3x3 orientation matrix of the camera

        Args:
            matrixIndex(int): Index ranging over all matrix elements
        Returns:
            float: The element of the matrix corresponding to the given index
        """
        return TT_CameraOrientationMatrix(self.index, matrixIndex)

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
            tangential1(float): Cameras second tangential distortion coefficient

         Raises:
            Exception: If the parameters could not be set
        """
        cdef float orientationp[9]
        for i in range(0,9):
            orientationp[i]=orientation[i]
        if not TT_CameraModel(self.index, x, y, z, orientationp, principleX, principleY,
                              focalLengthX, focalLengthY, kc1, kc2, kc3, tangential0, tangential1):
            raise Exception("Could Not Set Parameters")

    #TODO: Create camera mask class. See issue list on github
    #CAMERA MASKING
    def mask(self, buffer, int bufferSize):
        raise NotImplementedError
        cdef unsigned char * buffer=buffer          #buffer should be an integer array. See get_frame_buffer() below for example
        return TT_CameraMask(self.index, buffer, bufferSize)

    def set_mask(self, buffer, int bufferSize):
        raise NotImplementedError
        cdef unsigned char * buffer=buffer
        if not TT_SetCameraMask(self.index, buffer, bufferSize):
            raise Exception("Could Not Set Mask")

    def mask_info(self):
        raise NotImplementedError
        cdef int blockingMaskWidth=0, blockingMaskHeight=0, blockingMaskGrid=0
        if TT_CameraMaskInfo(self.index, blockingMaskWidth, blockingMaskHeight, blockingMaskGrid):
            return {'blockingMaskwidth':blockingMaskWidth,'blockingMaskHeight': blockingMaskHeight, 'blockingMaskGrid': blockingMaskGrid}
        else:
            raise Exception("Possibly Camera {0} Has No Mask".format(self.index))

    def clear_mask(self):
        raise NotImplementedError
        if not TT_ClearCameraMask(self.index):
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
        TT_CameraUndistort2DPoint(self.index, x, y)
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
        TT_CameraDistort2DPoint(self.index, x, y)
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
        TT_CameraBackproject(self.index, x, y, z, cameraX, cameraY)
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
        cdef float rayStartX=0, rayStartY=0, rayStartZ=0, rayEndX=0, rayEndY=0, rayEndZ=0
        if TT_CameraRay(self.index, x, y, rayStartX, rayStartY, rayStartZ, rayEndX, rayEndY, rayEndZ):
            return {'Xstart':rayStartX, 'Ystart':rayStartY, 'Zstart':rayStartZ,'Xend':rayEndX, 'Yend':rayEndY, 'Zend':rayEndZ}
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
        if TT_FrameCameraCentroid(markerIndex,self.index, x, y):
            return x, y
        else:
            raise Exception("Camera Not Contributing To 3D Position Of Marker {0}. \n Try Different Camera.".format(markerIndex))

    def get_frame_buffer(self):
        """Returns the cameras frame buffer

        This function fills the provided buffer with an image of what is in the camera view.
        The resulting image depends on what video mode the camera is in.
        If the camera is in grayscale mode, for example, a grayscale image is returned from this call.

        Returns:
            Frame buffer array

        Raises:
            Warnings: If after a video mode switch the incoming data mode has not switched yet
        """
        width, height=self.frame_resolution
        cdef np.ndarray[unsigned char, ndim=2] frame = np.empty((height, width), dtype='B')
        if not TT_CameraFrameBuffer(self.index, width, height, width, 8, &frame[0,0]):
                                  #(camera number, width in pixels, height in pixels, width in bytes, bits per pixel, buffer name)
                                  #  -> where width in bytes should equal width in pixels * bits per pixel / 8
            raise BufferError("Camera Frame Could Not Be Buffered")

        # When video mode is changed, it often takes a few frames before the data coming in is correct.
        # Give a warning if you can detect that happening.
        # TODO: Get instant video mode switching.
        # TODO: np.unique slows down the whole process considerably. Maybe only check framesize
        if len(frame) != self.frame_resolution[1]:
            warnings.warn("Frame size not correct for current video mode. Call motive.update().")

        unique_values = np.unique(frame)
        if self.video_mode in [self.OBJECT_MODE, self.SEGMENT_MODE]:
            if len(unique_values) > 2:
                warnings.warn("Frame contains video or precision data. Call motive.update().")

        else:
            if len(unique_values) == 2:
                warnings.warn("Frame contains object or segment data. Call motive.update().")

        return frame


    def frame_buffer_save_as_bmp(self, str filename):
        """Saves camera's frame buffer as a BMP image file

        Args:
            filename(str): The name of the image file the buffer will be saved to
        Raises:
            IOError: If the buffer has not been succesfully saved"""
        if not TT_CameraFrameBufferSaveAsBMP(self.index, filename):
            raise IOError("Camera Frame Buffer Not Successfully Saved")

#Functions To Set Camera Property Value, But W\O Possibility To Get Value
    def set_filter_switch(self, bool enableIRFilter):
        """Switch filter of camera

        Args:
            enableIRFilter(bool): True, IR filter is on. False, visible light.
        Raises:
            Exception: If camera could not switch filter
        """
        if not TT_SetCameraFilterSwitch(self.index, enableIRFilter):
            raise Exception("Could Not Switch Filter. Possibly Camera Has No IR Filter")

    def set_agc(self, bool enableAutomaticGainControl):
        """Switch gain control

        Args:
            enableAutomaticGainControl(bool): True, AGC is on. False,  manual GC.
        Raises:
            Exception: If camera could not enable AGC
        """
        if not TT_SetCameraAGC(self.index, enableAutomaticGainControl):
            raise Exception("Could Not Enable AGC. Possibly Camera Has No AGC")

    def set_aec(self, bool enableAutomaticExposureControl):
        """Switch exposure control

        Args:
            enableAutomaticExposureControl(bool): True, AEC is on. False,  manual EC.
        Raises:
            Exception: If camera could not enable AEC
        """
        if not TT_SetCameraAEC(self.index, enableAutomaticExposureControl):
            raise Exception("Could Not Enable AEC. Possibly Camera Has No AEC")

    def set_high_power(self, bool enableHighPowerMode):
        """Set camera IR LED to high power mode

        Args:
            enableHighPowerMode(bool): True, HPM is on. False,  manual LED intensity with max 15.
        Raises:
            Exception: If camera could not enable HPM
        """
        if not TT_SetCameraHighPower(self.index, enableHighPowerMode):
            raise Exception("Could Not Enable HighPowerMode. Possibly Camera Has No HighPowerMode")

    def set_mjpeg_high_quality(self, int mjpegQuality):                  #set to 8 minimum quality. set to 100 maximum quality. TODO: detailed range check
        """Set camera mjpeg video mode to higher quality

        Args:
            mjpegQuality(int): Value determines MJPEG quality
        Raises:
            Exception: If camera could not change quality
        """
        if not TT_SetCameraMJPEGHighQuality(self.index, mjpegQuality):
            raise Exception("Could Not Enable HighQuality. Possibly Camera Has No HighQuality For MJPEG")