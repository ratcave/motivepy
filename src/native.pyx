__author__ = 'Vash'

include "cnative.pxd"


#DECORATORS
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
    """Decorator to continually call a function until it stops raising a RuntimeWarning or until timeout."""
    import time
    def decorator_fun(func):
        def wrapper(*args, **kwargs):
            end_time = time.time() + secs_to_timeout
            while time.time() < end_time:
                try:
                    return func(*args, **kwargs)
                except RuntimeWarning:
                    pass
            else:
                raise RuntimeWarning("No Frames Available: Timed Out after {} seconds".format(secs_to_timeout))
        return wrapper
    return decorator_fun

@block_for_frame(secs_to_timeout=3)
@check_npresult
def update_single_frame():
    """
    Process incoming camera data
    """
    return TT_UpdateSingleFrame()

@block_for_frame(secs_to_timeout=3)
@check_npresult
def update():
    """
    Process incoming camera data. More than one frame
    """
    return TT_Update()

def autoupdate(func):
    def wrapper(*args, **kwargs):
        """Decorator, to call update() right after calling the function."""
        output = func(*args, **kwargs)
        update()
        return output
    return wrapper



#STARTUP / SHUTDOWN
@autoupdate
@check_npresult
def initialize():
    """Initialize the connection to Motive.  Done automatically upon importing the Python package."""
    return TT_Initialize()

@check_npresult
def shutdown():
    """
    shutdown library
    """
    return TT_Shutdown()


#RIGID BODY INTERFACE FILES
@check_npresult
def load_calibration(str calib_filename):
    """
    load calibration
    """
    return TT_LoadCalibration(calib_filename)

@check_npresult
def load_rigid_bodies(str load_bodies_file):
    """
    load rigid bodies
    """
    return TT_LoadRigidBodies(load_bodies_file)

@check_npresult
def save_rigid_bodies(str rigid_filename):
    """
    save rigid bodies
    """
    return TT_SaveRigidBodies(rigid_filename)

@check_npresult
def add_rigid_bodies(str rigid_bodies_file):
    """
    add rigid bodies
    """
    return TT_AddRigidBodies(rigid_bodies_file)

@check_npresult
def load_project(str project_file):
    """
    load project file
    """
    return TT_LoadProject(project_file)

@check_npresult
def save_project(str project_file):
    """
    save project file
    """
    return TT_SaveProject(project_file)

@check_npresult
def load_calibration_from_memory(buffername, int buffersize):
    assert isinstance(buffername, str), "Buffername Needs To Be String"
    cdef unsigned char * buff=buffername
    return TT_LoadCalibrationFromMemory(buff, buffersize)


#DATA STREAMING
@check_npresult
def stream_trackd(bool enabled):
    """
    Start/stop Trackd Stream.
    TrackD Streaming Engine: Streams rigid body data via the Trackd protocol
    """
    return TT_StreamTrackd(enabled)

@check_npresult
def stream_vrpn(bool enabled, int port=3883):
    """
    Start/stop VRPN Stream.
    VRPN Streaming Engine: Streams rigid body data via the VRPN protocol.
    VRPN Broadcast Port: Specifies the broadcast port for VRPN streaming.
    """
    return TT_StreamVRPN(enabled, port)

@check_npresult
def stream_np(bool enabled):
    """
    Start/stop NaturalPoint Stream
    """
    return TT_StreamNP(enabled)


#MARKERS
def get_frame_markers():
    """
    Returns list of all marker positions.
    """
    return tuple((TT_FrameMarkerX(i), TT_FrameMarkerY(i), TT_FrameMarkerZ(i)) for i in xrange(TT_FrameMarkerCount()))


# def unident_markers(int rigidBody_count):
#     """
#     returns a list of all markers which
#     are not in rigid Bodies
#     """
#     markers=frame_markers()
#     unimarkers=[]
#     imarkers=[]
#     for i in range (0,rigidBody_count):
#         for ik in rigidBody_markers(i):
#            imarkers.append(ik)
#            print "{}\n".format(ik)
#         print '\n{}\n\n'.format(imarkers)
#     for k in markers:
#         if k not in imarkers:
#            unimarkers.append(k)
#            print '{}\n'.format(k)
#     return unimarkers




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


#CAMERA GROUP SUPPORT
def camera_group_count():
    """
    Returns number of camera groups
    """
    return TT_CameraGroupCount()

def create_camera_group():
    """
    Add an additional group
    """
    if not TT_CreateCameraGroup():
        raise Exception("Could Not Create Camera Group")

def remove_camera_group(int groupIndex):
    """
    Remove a camera group (must be empty)
    """
    if not TT_RemoveCameraGroup(groupIndex):
        raise Exception("Could Not Remove. Check If Group Empty")

def set_group_shutter_delay(int groupIndex, int microseconds):
    """
    Set camera group's shutter delay
    """
    TT_SetGroupShutterDelay(groupIndex, microseconds)


#RIGID BODY CONTROL

#MARKER SIZE SETTINGS
@check_npresult
def set_camera_group_reconstruction(int groupIndex, bool enable):
    return TT_SetCameraGroupReconstruction(groupIndex, enable)


@check_npresult
def set_enabled_filter_switch(bool enabled):
    return TT_SetEnabledFilterSwitch(enabled)


def is_filter_switch_enabled():
    return TT_IsFilterSwitchEnabled()


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

def get_build_number():
    return TT_BuildNumber()