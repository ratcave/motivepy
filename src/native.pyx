"""Motive Native Module

This module features the functionality to load and save files
for camera settings, rigid bodies and markers.

Examples::

    >>>load_project("test.ttp")
    >>>update()
    >>>get_frame_markers()
    ((0.44324554, 0.65645343, 1.5665743), (0.23456576, 0.11568943, 0.04334536), (1.43445367, 1.23546491, 2.34356222))

"""

include "cnative.pxd"

from motive import utils


#STARTUP / SHUTDOWN
@utils.decorators.check_npresult
def _initialize():
    """Initializes the connection to the cameras

    This function is called automatically upon importing motive.
    """
    return TT_Initialize()


def shutdown():
    """Closes the connection to the cameras"""
    return utils.decorators.check_npresult(TT_Shutdown)()

@utils.decorators.block_for_frame(secs_to_timeout=3)
def update_single_frame():
    """Processes incoming camera data, grabs next frame in buffer"""
    return utils.decorators.check_npresult(TT_UpdateSingleFrame)()

@utils.decorators.block_for_frame(secs_to_timeout=3)
@utils.decorators.check_npresult
def update():
    """Processes incoming camera data. Grabs next frame in buffer if calling rate is similar to camera frame rate

    If calling rate is slower than camera frame rate, only grabs frames in intervals"""
    return TT_Update()


#RIGID BODY INTERFACE FILES
@utils.decorators._save_backup
@utils.decorators.check_npresult
def load_calibration(str file_name):
    """Loads camera calibration data from a file

    Note:
        The file should have the extension .cal
    Args:
        file_name(str): Name of the file
    """
    raise NotImplementedError
    utils.crash_avoidance.check_file_exists(file_name)
    utils.crash_avoidance.check_file_extension(file_name, '.cal')
    return TT_LoadCalibration(file_name)


@utils.decorators._save_backup
@utils.decorators.check_npresult
def load_rigid_bodies(str file_name):
    """Loads rigid body data from a file

    Note:
        The file should have the extension .tra
    Args:
        file_name(str): Name of the file
    """
    raise NotImplementedError
    utils.crash_avoidance.check_file_exists(file_name)
    utils.crash_avoidance.check_file_extension(file_name, '.tra')
    return TT_LoadRigidBodies(file_name)


@utils.decorators._save_backup
@utils.decorators.check_npresult
def save_rigid_bodies(str file_name):
    """Saves rigid body data to a file

    Note:
        The file should have the extension .tra
    Args:
        file_name(str): Name of the file
    """
    raise NotImplementedError
    utils.crash_avoidance.check_file_extension(file_name, '.tra')
    return TT_SaveRigidBodies(file_name)


@utils.decorators.check_npresult
def add_rigid_bodies(str file_name):
    """Adds rigid body data to an existing file

    Note:
        The file should have the extension .tra
    Args:
        file_name(str): Name of the file
    """
    raise NotImplementedError
    utils.crash_avoidance.check_file_extension(file_name, '.tra')
    utils.crash_avoidance.check_file_exists(file_name)
    return TT_AddRigidBodies(file_name)


@utils.decorators._save_backup
@utils.decorators.check_npresult
def load_project(str project_file=utils.backup_project_filename):
    """Loads the data of a project file

    E.g.: Camera calibration data, camera settings and rigid body data.
     Note:
        The file should have the extension .ttp
    Args:
        file_name(Optional[str]): Name of the file
            If left blank, will load the most recently loaded or saved project file
    """

    # Check File name and raise appropriate errors.
    utils.crash_avoidance.check_file_exists(project_file)
    utils.crash_avoidance.check_file_extension(project_file, '.ttp')

    # Load Project File
    return TT_LoadProject(project_file)


@utils.decorators.check_npresult
def _save_project(str project_file):
    """Saves project file

    Note:
        The file should have the extension .ttp
    Args:
        file_name(str): Name of the file
    Raises:
        IOError: If file has wrong extension
    """

    # Check File name and raise appropriate errors
    utils.crash_avoidance.check_file_extension(project_file, '.ttp')

    # Save Project File
    return TT_SaveProject(project_file)


@utils.decorators._save_backup
def save_project(str project_file):
    """Saves project file

    Note:
        The file should have the extension .ttp
    Args:
        file_name(str): Name of the file
    """
    _save_project(project_file)

#TODO: Find out how this works
@utils.decorators.check_npresult
def load_calibration_from_memory(buffer, int buffersize):
    raise NotImplementedError
    cdef unsigned char * buff=buffer         #buffer should be an integer array. See get_frame_buffer() in camera.pyx for example
    return TT_LoadCalibrationFromMemory(buff, buffersize)

#DATA STREAMING
@utils.decorators.check_npresult
def stream_trackd(bool enabled):
    """Start/stop Trackd Stream

    TrackD Streaming Engine: Streams rigid body data via the Trackd protocol.
    Args:
        enabled(bool): True to start Trackd Stream. False to stop it.
    """
    return TT_StreamTrackd(enabled)

@utils.decorators.check_npresult
def stream_vrpn(bool enabled, int port=3883):
    """Start/stop VRPN Stream

    VRPN Streaming Engine: Streams rigid body data via the VRPN protocol.
    VRPN Broadcast Port: Specifies the broadcast port for VRPN streaming.

    Args:
        enabled(bool): True to start VRPN Stream. False to stop it.
        port(Optional[int]): Encodes the broadcast port
    """
    return TT_StreamVRPN(enabled, port)

@utils.decorators.check_npresult
def stream_np(bool enabled):
    """Start/stop NaturalPoint Stream

    Args:
        enabled(bool): True to start NaturalPoint Stream. False to stop it.
    """
    return TT_StreamNP(enabled)


#MARKERS
def get_frame_markers():
    """Returns a tuple containing all tuples of 3D marker positions"""
    return tuple((TT_FrameMarkerX(i), TT_FrameMarkerY(i), TT_FrameMarkerZ(i)) for i in xrange(TT_FrameMarkerCount()))

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

def frame_marker_label(marker_index):
    """Returns marker label object

    This object holds a unique ID for every marker.
    """
    ID=_markID()
    cdef cUID label=TT_FrameMarkerLabel(marker_index)
    ID.thisptr=&label
    return ID

def frame_time_stamp():
    """Returns time stamp of frame in seconds"""
    return TT_FrameTimeStamp()

def flush_camera_queues():
    """In the event that you are tracking a very high number of 2D and/or 3D markers (hundreds of 3D markers),
    and you find that the data you're getting out has sufficient latency you can call flush_camera_queues()
    to catch up before calling update(). Ideally, after calling flush_camera_queues() you'll want to not
    call it again until after update() returns a frame again.
    """
    TT_FlushCameraQueues()


#TODO: Check what the below functions actually do, then remove the not implemented error
#MARKER SIZE SETTINGS
@utils.decorators.check_npresult
def set_camera_group_reconstruction(int groupIndex, bool enable):
    raise NotImplementedError
    return TT_SetCameraGroupReconstruction(groupIndex, enable)

@utils.decorators.check_npresult
def set_enabled_filter_switch(bool enabled):
    raise NotImplementedError
    return TT_SetEnabledFilterSwitch(enabled)

def is_filter_switch_enabled():
    raise NotImplementedError
    return TT_IsFilterSwitchEnabled()


#ADDITIONAL FUNCTIONALITY
def set_frame_id_based_timing(bool enable):
    raise NotImplementedError
    return TT_SetFrameIDBasedTiming(enable)

def set_suppress_out_of_order(bool enable):
    raise NotImplementedError
    return TT_SetSuppressOutOfOrder(enable)

@utils.decorators.check_npresult
def orient_tracking_bar(float positionX, float positionY, float positionZ,
                        float orientationX, float orientationY, float orientationZ, float orientationW):
    raise NotImplementedError
    return TT_OrientTrackingBar(positionX, positionY, positionZ,
                                orientationX, orientationY, orientationZ, orientationW)

def get_build_number():
    """Returns multiple digit integer number"""
    raise NotImplementedError()
    return TT_BuildNumber()