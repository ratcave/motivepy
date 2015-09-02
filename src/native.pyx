__author__ = 'Vash'

# means comment from official SDK API (or Cython page) ## means comment from me

include "cnative.pxd"


#STARTUP / SHUTDOWN
def initialize():
    """initialize library"""
    return TT_Initialize()

def shutdown():
    """shutdown library"""
    return TT_Shutdown()

#RIGID BODY INTERFACE ##FILES
def load_calibration(calib_filename):
    """load calibration"""
    assert type(calib_filename) is str, "Argument should be filename, i.e. a string literal"
    return TT_LoadCalibration(calib_filename)

def load_rigid_bodies(load_bodies_file):
    """load rigid bodies"""
    assert type(load_bodies_file) is str, "Argument should be filename, i.e. a string literal"
    return TT_LoadRigidBodies(load_bodies_file)

def save_rigid_bodies(rigid_filename):
    """save rigid bodies"""
    assert type(rigid_filename) is str, "Argument should be filename, i.e. a string literal"
    return TT_SaveRigidBodies(rigid_filename)

def add_rigid_bodies(rigid_bodies_file):
    """add rigid bodies"""
    assert type(rigid_bodies_file) is str, "Argument should be filename, i.e. a string literal"
    return TT_AddRigidBodies(rigid_bodies_file)

def load_project(project_file):
    """load project file"""
    assert type(project_file) is str, "Argument should be filename, i.e. a string literal"
    return TT_LoadProject(project_file)

def save_project(project_file):
    """save project file"""
    assert type(project_file) is str, "Argument should be filename, i.e. a string literal"
    return TT_SaveProject(project_file)

def load_calibration_from_memory(buffername,int buffersize):
    assert type (buffername) is str, "Argument should be buffername, i.e. a string literal"
    cdef unsigned char * buffer=buffername
    return TT_LoadCalibrationFromMemory(buffer,buffersize)

def update():
    """Process incoming camera data"""
    return TT_Update()

def update_single_frame():
    """Process incoming camera data"""
    return TT_UpdateSingleFrame()


#DATA STREAMING
def stream_trackd(bool enabled):
    """Start/stop Trackd Stream
       TrackD Streaming Engine: Streams rigid body data via the Trackd protocol"""
    return TT_StreamTrackd(enabled)

def stream_vrpn(bool enabled, int port):
    """Start/stop VRPN Stream
       VRPN Streaming Engine: Streams rigid body data via the VRPN protocol.
       VRPN Broadcast Port: Specifies the broadcast port for VRPN streaming. (Default: 3883)"""
    return TT_StreamVRPN(enabled, port)

def stream_np(bool enabled):
    """Start/stop NaturalPoint Stream"""
    return TT_StreamNP(enabled)


#FRAME
def frame_marker_count():
    """Returns Frame Markers Count"""
    return TT_FrameMarkerCount()

def frame_marker_x(int index):
    """Returns X Coord of Marker"""
    return TT_FrameMarkerX(index)

def frame_marker_y(int index):
    """Returns Y Coord of Marker"""
    return TT_FrameMarkerY(index)

def frame_marker_z(int index):
    """Returns Z Coord of Marker"""
    return TT_FrameMarkerZ(index)

def frame_time_stamp():
    """Time Stamp of Frame (seconds"""
    return TT_FrameTimeStamp()

def frame_camera_centroid(int index, int cameraIndex):
    """Returns true if the camera is contributing to this 3D marker.
       It also returns the location of the 2D centroid that is reconstructing to this 3D marker"""
    cdef float x,y
    xp=&x
    yp=&y
    return TT_FrameCameraCentroid(index,cameraIndex, x, y)
    assert TT_FrameCameraCentroid(index,cameraIndex, x, y), "Camera is not contributing to the 3D position of this marker"
    print "\n \n 2D x-position as seen from camera %i is %d" % (cameraIndex, x)
    print "\n 2D y-position is %d" % y

def flush_camera_queues():
    """In the event that you are tracking a very high number of 2D and/or 3D markers (hundreds of 3D markers),
    and you find that the data you're getting out has sufficient latency you can call TT_FlushCameraQueues()
    to catch up before calling TT_Update(). Ideally, after calling TT_FlushCameraQueues() you'll want to not
    call it again until after TT_Update() returns 0"""
    TT_FlushCameraQueues()
    print "Flushed"












def set_camera_settings(camindex, videotype, exposure, threshold, intensity):
    return TT_SetCameraSettings(camindex, videotype, exposure, threshold, intensity)

def set_camera_group(camindex, camgroupindex):
    TT_SetCameraGroup(camindex, camgroupindex)
    print "set camera group"