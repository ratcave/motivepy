"""Motive Native Module

This module features the functionality to load and save files
for camera settings, rigid bodies and markers.

Examples::

    >>>load_profile("test.motive")
    >>>update()
    >>>get_frame_markers()
    ((0.44324554, 0.65645343, 1.5665743), (0.23456576, 0.11568943, 0.04334536), (1.43445367, 1.23546491, 2.34356222))

"""
from __future__ import absolute_import

include "cnative.pxd"

from . import decorators, crash_avoidance

# from python_object cimport PyObject

cdef extern from *:
    wchar_t* PyUnicode_AsWideCharString(object, Py_ssize_t *size)

   

def check_npresult(func):
    """Decorator that checks if the output of a function matches the Motive Error Values, and raises a Python error if so

    Note:
        Should decorate every Motive API function returning a NPResult type.
    """
    error_dict = {kApiResult_FileNotFound:  (IOError, "File Not Found"),
                  kApiResult_LoadFailed:  (Exception, "Load Failed"),
                  kApiResult_SaveFailed:  (Exception, "Save Failed"),
                  kApiResult_Failed:  (Exception, "Failed"),
                  kApiResult_InvalidFile:  (IOError, "Invalid File"),
                  kApiResult_InvalidLicense: (EnvironmentError, "Invalid License"),
                  kApiResult_NoFrameAvailable: (RuntimeWarning, "No Frames Available"),
                  kApiResult_TooFewMarkers: (RuntimeWarning, "Too Few Markers")}
    def wrapper(*args, **kwargs):
        npresult = func(*args, **kwargs)
        if npresult != kApiResult_Success:
            error, msg = error_dict[npresult]
            raise error(msg)
    return wrapper


#STARTUP / SHUTDOWN
@check_npresult
def initialize():
    """Initializes the connection to the cameras"""
    return Initialize()

@check_npresult
def shutdown():
    """Closes the connection to the cameras"""
    Shutdown()

@decorators.block_for_frame(secs_to_timeout=3)
def update_single_frame():
    """Processes incoming camera data, grabs next frame in buffer"""
    return check_npresult(UpdateSingleFrame)()


@decorators.block_for_frame(secs_to_timeout=3)
def update():
    """Processes incoming camera data. Grabs next frame in buffer if calling rate is similar to camera frame rate

    If calling rate is slower than camera frame rate, only grabs frames in intervals"""
    return check_npresult(Update)()


#RIGID BODY INTERFACE FILES
@decorators._save_backup
def load_calibration(str file_name):
    """Loads camera calibration data from a file

    Note:
        The file should have the extension .cal
    Args:
        file_name(str): Name of the file
    """
    # raise NotImplementedError
    crash_avoidance.check_file_exists(file_name)
    crash_avoidance.check_file_extension(file_name, '.cal')
    cdef int cameraCount = 0
    cdef int* value = &cameraCount
    cdef Py_ssize_t length
    LoadCalibration(PyUnicode_AsWideCharString(file_name, &length), value)

@decorators._save_backup
def load_rigid_bodies(str file_name):
    """Loads rigid body data from a file

    Note:
        The file should have the extension .tra
    Args:
        file_name(str): Name of the file
    """
    # raise NotImplementedError
    crash_avoidance.check_file_exists(file_name)
    crash_avoidance.check_file_extension(file_name, '.tra')
    cdef Py_ssize_t length
    return LoadRigidBodies(PyUnicode_AsWideCharString(file_name, &length))


@decorators._save_backup
def save_rigid_bodies(str file_name):
    """Saves rigid body data to a file

    Note:
        The file should have the extension .tra
    Args:
        file_name(str): Name of the file
    Note:
        Not Implemented!
    """
    raise NotImplementedError
    crash_avoidance.check_file_extension(file_name, '.tra')
    return check_npresult(SaveRigidBodies)(file_name.encode('UTF-8'))


@check_npresult
def add_rigid_bodies(str file_name):
    """Adds rigid body data to an existing file

    Note:
        The file should have the extension .tra
    Args:
        file_name(str): Name of the file
    Note:
        Not Implemented!
    """
    raise NotImplementedError
    crash_avoidance.check_file_extension(file_name, '.tra')
    crash_avoidance.check_file_exists(file_name)
    return AddRigidBodies(file_name.encode('UTF-8'))


#@decorators._save_backup
def load_profile(str profile_file=crash_avoidance.backup_profile_filename):
    """Loads the data of a profile file

    E.g.: Camera calibration data, camera settings and rigid body data.
     Note:
        The file should have the extension .motive
    Args:
        file_name(Optional[str]): Name of the file
            If left blank, will load the most recently loaded or saved profile file
    """
    
    # Check File name and raise appropriate errors.
    crash_avoidance.check_file_exists(profile_file)
    crash_avoidance.check_file_extension(profile_file, '.motive')
    
    # Load Profile File
    #return check_npresult(LoadProfile)(profile_file.encode('UTF-8'))
    cdef Py_ssize_t length
    LoadProfile(PyUnicode_AsWideCharString(profile_file, &length))

def _save_profile(str profile_file):
    """Saves profile file

    Note:
        The file should have the extension .motive
    Args:
        file_name(str): Name of the file
    Raises:
        IOError: If file has wrong extension
    """

    # Check File name and raise appropriate errors
    crash_avoidance.check_file_extension(profile_file, '.motive')

    # Save Profile File
    cdef Py_ssize_t length
    return SaveProfile(PyUnicode_AsWideCharString(profile_file, &length))


@decorators._save_backup
def save_profile(str profile_file):
    """Saves profile file

    Note:
        The file should have the extension .motive
    Args:
        file_name(str): Name of the file
    """
    _save_profile(profile_file)

#TODO: Find out how this works
def load_calibration_from_memory(buffer, int buffersize):
    """Note: Not Implemented!"""
    raise NotImplementedError
    cdef unsigned char * buff=buffer         #buffer should be an integer array. See get_frame_buffer() in camera.pyx for example
    return check_npresult(LoadCalibrationFromMemory)(buff, buffersize)



#MARKERS
def get_frame_markers():
    """Returns a tuple containing all tuples of 3D marker positions"""
    cdef float x=0, y=0, z=0
    cdef list coords = []
    for i in range(MarkerCount()):
        # Call the function; it returns True on success (assumes success for all)
        MarkerXYZ(i, x, y, z)
        coords.append((x, y, z))

    return tuple(coords)
    #return tuple((FrameMarkerX(i), FrameMarkerY(i), FrameMarkerZ(i)) for i in xrange(FrameMarkerCount()))

cdef class _markID:
    cdef cUID *thisptr            # hold a C++ instance which we're wrapping

    def __cinit__(self):
        """returns an uninitialized marker label object"""
        self.thisptr = NULL

    @property
    def low_bits(self):
        """19 digit integer number ending with a capital L (20 digits)"""
        return self.thisptr.LowBits()

    @property
    def high_bits(self):
        """19 digit integer number ending with a capital L(20 digits)"""
        return self.thisptr.HighBits()


def frame_time_stamp():
    """Returns time stamp of frame in seconds"""
    return FrameTimeStamp()

def flush_camera_queues():
    """In the event that you are tracking a very high number of 2D and/or 3D markers (hundreds of 3D markers),
    and you find that the data you're getting out has sufficient latency you can call flush_camera_queues()
    to catch up before calling update(). Ideally, after calling flush_camera_queues() you'll want to not
    call it again until after update() returns a frame again.
    """
    FlushCameraQueues()


#TODO: Check what the below functions actually do, then remove the not implemented error
#MARKER SIZE SETTINGS
def set_camera_group_reconstruction(int groupIndex, bool enable):
    """Note: Not Implemented!"""
    raise NotImplementedError
    return check_npresult(SetCameraGroupReconstruction)(groupIndex, enable)

def set_enabled_filter_switch(bool enabled):
    """Note: Not Implemented!"""
    raise NotImplementedError
    return check_npresult(SetEnabledFilterSwitch)(enabled)

def is_filter_switch_enabled():
    """Note: Not Implemented!"""
    raise NotImplementedError
    return IsFilterSwitchEnabled()


#ADDITIONAL FUNCTIONALITY
def set_frame_id_based_timing(bool enable):
    """Note: Not Implemented!"""
    raise NotImplementedError
    return SetFrameIDBasedTiming(enable)

def set_suppress_out_of_order(bool enable):
    """Note: Not Implemented!"""
    raise NotImplementedError
    return SetSuppressOutOfOrder(enable)

def orient_tracking_bar(float positionX, float positionY, float positionZ,
                        float orientationX, float orientationY, float orientationZ, float orientationW):
    """Note: Not Implemented!"""
    raise NotImplementedError
    return check_npresult(OrientTrackingBar)(positionX, positionY, positionZ,
                                orientationX, orientationY, orientationZ, orientationW)

def get_build_number():
    """Returns multiple digit integer number"""
    return BuildNumber()