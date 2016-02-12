

include "cnative.pxd"
cimport cython
from motive import utils

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
    TT_SetGroupShutterDelay(groupIndex, microseconds)

@utils.decorators.check_npresult
def get_camera_group_point_cloud_settings(groupIndex, CameraGroupPointCloudSettings Settings):
    """Gets the settings of the camera group and sets them in the CameraGroupPointCloudSettings object)

    """
    TT_CameraGroupPointCloudSettings   (groupIndex, Settings.obj[0])

@utils.decorators.check_npresult
def set_camera_group_point_cloud_settings(groupIndex, CameraGroupPointCloudSettings Settings):
    """Sets the settings in the camera group to the settings of the CameraGroupPointCloudSettings object)

    """
    TT_SetCameraGroupPointCloudSettings(groupIndex, Settings.obj[0]) #Cannot assign type 'cCameraGroupPointCloudSettings *' to 'cCameraGroupPointCloudSettings'


cdef class CameraGroupPointCloudSettings:
      cdef cCameraGroupPointCloudSettings *obj

      def __cinit__(self):
          self.obj=new cCameraGroupPointCloudSettings()
          # print([fun for fun in locals() if fun[0] == 'e' and fun[1].isupper()])

      def __dealloc__(self):
          del self.obj

      def set_bool_parameter(self, value):
          assert self.obj.SetBoolParameter(ePCCalculateDiameter, value ),"Type of setting is of different type than value"

      def set_double_parameter( self, which , value ):
          assert self.obj.SetDoubleParameter( ePCResidual, value ),"Type of setting is of different type than value"

      def set_long_parameter( self,value ):
          assert self.obj.SetLongParameter( eShutterDelay, value ),"Type of setting is of different type than value" #eShutterDelay getter and setter for shutter delay?

      def get_bool_parameter(self):
          cdef bool value=True
          assert self.obj.BoolParameter(ePCCalculateDiameter, value),"Type of setting is of different type than value"
          return value

      def get_double_parameter(self):
          cdef double value=0
          assert self.obj.DoubleParameter(ePCResidual, value),"Type of setting is of different type than value"
          return value

      def get_long_parameter(self):
          cdef long value=0
          assert self.obj.LongParameter(eShutterDelay, value),"Type of setting is of different type than value"
          return value
