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
    #cdef const char * file=calib_filename
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









def set_camera_settings(camindex, videotype, exposure, threshold, intensity):
    return TT_SetCameraSettings(camindex, videotype, exposure, threshold, intensity)

def set_camera_group(camindex, camgroupindex):
    TT_SetCameraGroup(camindex, camgroupindex)
    print "set camera group"