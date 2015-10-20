__author__ = 'Vash'

include "cnative.pxd"

from os import path
from motive import utils

# Cython Decorators
def _save_backup(func):
    def wrapper(*args, **kwargs):
        func(*args, **kwargs)
        _save_project(utils.backup_project_filename)
    return wrapper

#STARTUP / SHUTDOWN
@utils.decorators.check_npresult
def _initialize():
    """Initialize the connection to Motive.  Done automatically upon importing the Python package."""
    return TT_Initialize()

@utils.decorators.check_npresult
def shutdown():
    """
    shutdown library
    """
    return TT_Shutdown()

@utils.decorators.block_for_frame(secs_to_timeout=3)
@utils.decorators.check_npresult
def update_single_frame():
    """Process incoming camera data"""
    return TT_UpdateSingleFrame()

@utils.decorators.block_for_frame(secs_to_timeout=3)
@utils.decorators.check_npresult
def update():
    """Process incoming camera data. More than one frame"""
    return TT_Update()


#RIGID BODY INTERFACE FILES
@_save_backup
@utils.decorators.check_npresult
def load_calibration(str file_name):
    """
    load calibration
    """
    utils.crash_avoidance.check_file_exists(file_name)
    utils.crash_avoidance.check_file_extension(file_name, '.cal')
    return TT_LoadCalibration(file_name)

@_save_backup
@utils.decorators.check_npresult
def load_rigid_bodies(str file_name):
    """
    load rigid bodies
    """
    utils.crash_avoidance.check_file_exists(file_name)
    utils.crash_avoidance.check_file_extension(file_name, '.tra')
    return TT_LoadRigidBodies(file_name)

@_save_backup
@utils.decorators.check_npresult
def save_rigid_bodies(str file_name):
    """
    save rigid bodies
    """
    utils.crash_avoidance.check_file_extension(file_name, '.tra')
    return TT_SaveRigidBodies(file_name)


@utils.decorators.check_npresult
def add_rigid_bodies(str file_name):
    """
    add rigid bodies
    """
    utils.crash_avoidance.check_file_extension(file_name, '.tra')
    utils.crash_avoidance.check_file_exists(file_name)
    return TT_AddRigidBodies(file_name)




@_save_backup
@utils.decorators.check_npresult
def load_project(str project_file=utils.backup_project_filename):
    """Loads a Motive .ttp Project File.  If left blank, will load the most recently worked on Project file."""

    # Check File name and raise appropriate errors.
    utils.crash_avoidance.check_file_exists(project_file)
    utils.crash_avoidance.check_file_extension(project_file, '.ttp')

    # Load Project File
    return TT_LoadProject(project_file)


#@utils.decorators.check_npresult
def _save_project(str project_file):
    """Saves project file."""

    # Check File name and raise appropriate errors
    utils.crash_avoidance.check_file_extension(project_file, '.ttp')

    # Save Project File
    return TT_SaveProject(project_file)

save_project = _save_backup(_save_project)  #  Saves a project file, and  also saves a backup version in the app data directory.

@utils.decorators.check_npresult
def load_calibration_from_memory(buffername, int buffersize):
    assert isinstance(buffername, str), "Buffername Needs To Be String"
    cdef unsigned char * buff=buffername
    return TT_LoadCalibrationFromMemory(buff, buffersize)


#DATA STREAMING
@utils.decorators.check_npresult
def stream_trackd(bool enabled):
    """
    Start/stop Trackd Stream.
    TrackD Streaming Engine: Streams rigid body data via the Trackd protocol
    """
    return TT_StreamTrackd(enabled)

@utils.decorators.check_npresult
def stream_vrpn(bool enabled, int port=3883):
    """
    Start/stop VRPN Stream.
    VRPN Streaming Engine: Streams rigid body data via the VRPN protocol.
    VRPN Broadcast Port: Specifies the broadcast port for VRPN streaming.
    """
    return TT_StreamVRPN(enabled, port)

@utils.decorators.check_npresult
def stream_np(bool enabled):
    """
    Start/stop NaturalPoint Stream
    """
    return TT_StreamNP(enabled)


#MARKERS
def get_frame_markers():
    """Returns list of all marker positions"""
    return tuple((TT_FrameMarkerX(i), TT_FrameMarkerY(i), TT_FrameMarkerZ(i)) for i in xrange(TT_FrameMarkerCount()))

cdef class markID:
    cdef cUID *thisptr            # hold a C++ instance which we're wrapping

    def __cinit__(self):
        self.thisptr =NULL
    @property
    def low_bits(self):
        return self.thisptr.LowBits()

    @property
    def high_bits(self):
        return self.thisptr.HighBits()


def frame_marker_label(marker_index):
    """"
    returns marker label (cUID) class object
    """
    ID=markID()
    cdef cUID label=TT_FrameMarkerLabel(marker_index)
    ID.thisptr=&label
    return ID

def frame_time_stamp():
    """
    Time Stamp of Frame (seconds)
    """
    return TT_FrameTimeStamp()

def flush_camera_queues():
    """
    In the event that you are tracking a very high number of 2D and/or 3D markers (hundreds of 3D markers),
    and you find that the data you're getting out has sufficient latency you can call TT_FlushCameraQueues()
    to catch up before calling TT_Update(). Ideally, after calling TT_FlushCameraQueues() you'll want to not
    call it again until after TT_Update() returns 0
    """
    TT_FlushCameraQueues()


#MARKER SIZE SETTINGS
@utils.decorators.check_npresult
def set_camera_group_reconstruction(int groupIndex, bool enable):
    return TT_SetCameraGroupReconstruction(groupIndex, enable)


@utils.decorators.check_npresult
def set_enabled_filter_switch(bool enabled):
    return TT_SetEnabledFilterSwitch(enabled)


def is_filter_switch_enabled():
    return TT_IsFilterSwitchEnabled()


#ADDITIONAL FUNCTIONALITY
def set_frame_id_based_timing(bool enable):
    return TT_SetFrameIDBasedTiming(enable)

def set_suppress_out_of_order(bool enable):
    return TT_SetSuppressOutOfOrder(enable)

@utils.decorators.check_npresult
def orient_tracking_bar(float positionX, float positionY, float positionZ,
                        float orientationX, float orientationY, float orientationZ, float orientationW):
    return TT_OrientTrackingBar(positionX, positionY, positionZ,
                                orientationX, orientationY, orientationZ, orientationW)

def get_build_number():
    return TT_BuildNumber()