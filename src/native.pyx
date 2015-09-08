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

def frame_camera_centroid(int index, int cameraIndex, float x, float y):
    """Returns true if the camera is contributing to this 3D marker.
       It also returns the location of the 2D centroid that is reconstructing to this 3D marker"""
    if TT_FrameCameraCentroid(index,cameraIndex, x, y):
        print "\n \n 2D x-position as seen from camera %i is %f" % (cameraIndex, x)
        print "\n 2D y-position is %f" % y
    else:
        print "Camera is not contributing to the 3D position of this marker"
        
def flush_camera_queues():
    """In the event that you are tracking a very high number of 2D and/or 3D markers (hundreds of 3D markers),
    and you find that the data you're getting out has sufficient latency you can call TT_FlushCameraQueues()
    to catch up before calling TT_Update(). Ideally, after calling TT_FlushCameraQueues() you'll want to not
    call it again until after TT_Update() returns 0"""
    TT_FlushCameraQueues()
    print "Flushed"


#RIGID BODY CONTROL
def set_rigid_body_user_data(int index, int ID):
    """Set RigidBodies User Data"""
    TT_SetRigidBodyUserData(index,ID)
    print "Set"

def rigid_body_user_data(int index):
    """Get RigidBodies User Data"""
    print "Rigid body ID: %i" %  TT_RigidBodyUserData(index)

def rigid_body_name(int index):
    """Returns RigidBody Name"""
    print "%s" % TT_RigidBodyName(index)

def set_rigid_body_enabled(int index, bool enabled):
    """Set tracking """
    TT_SetRigidBodyEnabled(index, enabled)

def rigid_body_enabled(int index):
    """Get tracking"""
    return TT_RigidBodyEnabled(index)

def is_rigid_body_tracked(int index):
    """Is rigid body currently tracked"""
    if TT_IsRigidBodyTracked(index):
        print "Yes"
    else:
        print "No"

def rigid_body_location(int index, float x, float y, float z,
                        float qx, float qy, float qz, float qw,
                        float yaw, float pitch, float roll):
    """##Not sure if this function sets or gets the location.
    If it returns values different from the ones you entered,
    the function gets the location as computed by Motive.
    Otherwise it is for manually setting the location."""
    TT_RigidBodyLocation(index,  &x, &y, &z,  &qx, &qy, &qz, &qw, &yaw, &pitch, &roll)
    print "The position of rigid body %i is x=%f, y=%f, z=%f. \n" % (x, y, z)
    print "Orientation in quaternions is qx=%f, qy=%f, qz=%f, qw=%f. \n" % (qx, qy, qz, qw)
    print "Yaw is %f, pitch is %f, roll is %f." % (yaw, pitch, roll)

def rigid_body_translate_pivot(int index, float x, float y, float z):
    """Rigid Body Pivot-Point Translation: Sets a translation offset for the centroid of the rigid body.
    Reported values for the location of the rigid body, as well as the 3D visualization, will be shifted
    by the amount provided in the fields on either the X, Y, or Z axis. Values are entered in meters. """
    return   TT_RigidBodyTranslatePivot(index, x, y, z)

def rigid_body_reset_orientation(int index):
    """Reset orientation to match the current tracked orientation
    of the rigid body"""
    TT_RigidBodyResetOrientation(index)

def clear_rigid_body_list():
    """Clear all rigid bodies"""
    TT_ClearRigidBodyList()
    print "Cleared"

def remove_rigid_body(int index):
    """Remove single rigid body"""
    return TT_RemoveRigidBody(index)

def rigid_body_marker_count(int index):
    """Get marker count"""
    return TT_RigidBodyMarkerCount(index)

def rigid_body_marker(int rigidIndex, int markerIndex, float x, float y, float z):
    """Get rigid body marker.
    ##Not sure if this function sets or gets the location.
    If it returns values different from the ones you entered,
    the function gets the location as computed by Motive.
    Otherwise it is for manually setting the location."""
    TT_RigidBodyMarker(rigidIndex, markerIndex, &x, &y, &z)
    print "The position of rigid body's %i marker %i, is x=%f, y=%f, z=%f. \n" % (rigidIndex, markerIndex, x, y, z)

def rigid_body_point_cloud_marker(int rigidIndex, int markerIndex, bool tracked, float x, float y, float z):
    """ Get corresponding point cloud marker
    If tracked is false, there is no corresponding point cloud marker.
    """
    TT_RigidBodyPointCloudMarker(rigidIndex, markerIndex, tracked, x, y, z)
    if tracked:
        print "The point cloud markers position is x=%f, y=%f, z=%f" % (x,y,z)
    else:
        print "There is no corresponding point cloud marker"

def create_rigid_body(str name, int id, int markerCount, markerList):
    """Create a rigid body based on the marker count and marker list provided.
    The marker list is expected to contain a list of marker coordinates in the order:
    x1,y1,z1,x2,y2,z2,...xN,yN,zN."""
    cdef float markerListp[1000]
    assert len(markerList)<=1000, "Due to need of const C array size, markerList max items=1000. \n Please resize const in native.pyx"
    for i in range(0,len(markerList)):
        markerListp[i]=markerList[i]

    return TT_CreateRigidBody(name, id, markerCount, markerListp)











def set_camera_settings(camindex, videotype, exposure, threshold, intensity):
    return TT_SetCameraSettings(camindex, videotype, exposure, threshold, intensity)

def set_camera_group(camindex, camgroupindex):
    TT_SetCameraGroup(camindex, camgroupindex)
    print "set camera group"