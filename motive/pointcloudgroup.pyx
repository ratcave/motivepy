"""Motive Camera Module

This module features the functionality to create and edit
camera groups as well as to get and set their respective
point cloud settings. It is basically made up of one large
CameraGroupPointCloudSettings class.
Once a CameraGroupPointCloudSettings object has been created,
the settings it stores can be adapted and finally set
as the new point cloud settings of the camera group.

Examples::

    >>>settings=CameraGroupPointCloudSettings()
    >>>settings.object_filter_type=2
    >>>set_camera_group_point_cloud_settings(0,settings)

"""
from __future__ import absolute_import

include "cnative.pxd"
cimport cython
from . import native

#CAMERA GROUP SUPPORT
def camera_group_count():
    """Returns number of camera groups"""
    return CameraGroupCount()

def create_camera_group():
    """Adds an additional camera group

    Raises:
        Exception: If a new camera group could not be created
    """
    if not CreateCameraGroup():
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
    if not RemoveCameraGroup(groupIndex):
        raise Exception("Could Not Remove. Check If Group Empty")


def set_group_shutter_delay(int groupIndex, int microseconds):
    """Set camera group's shutter delay

    Args:
        groupIndex (int): The index of the camera group
        microseconds (int): The time between opening of shutter and capture of frame
    """
    SetGroupShutterDelay(groupIndex, microseconds)

#TODO: Check if this actually works for a groupindex different from 0 or more than one group
@native.check_npresult
def get_camera_group_point_cloud_settings(groupIndex, CameraGroupPointCloudSettings Settings):
    """Gets the settings of the camera group and sets them in the CameraGroupPointCloudSettings object

    Args:
        groupIndex (int): The index of the camera group to get settings from
        Settings (CameraGroupPointCloudSetting): The object in which the settings are to be set
    """
    return CameraGroupPointCloudSettings(groupIndex, Settings.obj[0])

@native.check_npresult
def set_camera_group_point_cloud_settings(groupIndex, CameraGroupPointCloudSettings Settings):
    """Sets the settings in the camera group to the settings of the CameraGroupPointCloudSettings object

    Args:
        groupIndex (int): The index of the camera group to set settings in
        Settings (CameraGroupPointCloudSetting): The object in which holds the settings are to be set
    """
    return SetCameraGroupPointCloudSettings(groupIndex, Settings.obj[0]) #Cannot assign type 'cCameraGroupPointCloudSettings *' to 'cCameraGroupPointCloudSettings'


cdef class CameraGroupPointCloudSettings:
      cdef cCameraGroupPointCloudSettings *obj

      def __cinit__(self):
          self.obj=new cCameraGroupPointCloudSettings()
          # print([fun for fun in locals() if fun[0] == 'e' and fun[1].isupper()])

      def __dealloc__(self):
          del self.obj

      #Following are all point cloud reconstruction settings which can be set and get
      #1. Boolean
      #Note that properties in cython classes that wrap C++ classes have a different syntax:
      property resolve_point_cloud:
        """bool"""
        def __get__(self):
            cdef bool value=True
            assert self.obj.BoolParameter(eResolvePointCloud, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetBoolParameter(eResolvePointCloud, value ),"Type of setting is of different type than value"

      property show_cameras:
        """bool"""
        def __get__(self):
            cdef bool value=True
            assert self.obj.BoolParameter(eShowCameras, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetBoolParameter(eShowCameras, value ),"Type of setting is of different type than value"

      property show_reconstruction_bounds:
        """bool"""
        def __get__(self):
            cdef bool value=True
            assert self.obj.BoolParameter(eShowReconstructionBounds, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetBoolParameter(eShowReconstructionBounds, value ),"Type of setting is of different type than value"

      property bound_reconstruction:
        """bool"""
        def __get__(self):
            cdef bool value=True
            assert self.obj.BoolParameter(eBoundReconstruction, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetBoolParameter(eBoundReconstruction, value ),"Type of setting is of different type than value"

      property show_capture_volume:
        """bool"""
        def __get__(self):
            cdef bool value=True
            assert self.obj.BoolParameter(eShowCaptureVolume, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetBoolParameter(eShowCaptureVolume, value ),"Type of setting is of different type than value"

      property show_3D_markers:
        """bool"""
        def __get__(self):
            cdef bool value=True
            assert self.obj.BoolParameter(eShow3DMarkers, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetBoolParameter(eShow3DMarkers, value ),"Type of setting is of different type than value"

      property show_camera_FOV:
        """bool"""
        def __get__(self):
            cdef bool value=True
            assert self.obj.BoolParameter(eShowCameraFOV, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetBoolParameter(eShowCameraFOV, value ),"Type of setting is of different type than value"

      property rank_rays:
        """bool"""
        def __get__(self):
            cdef bool value=True
            assert self.obj.BoolParameter(eRankRays, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetBoolParameter(eRankRays, value ),"Type of setting is of different type than value"

      property calculate_diameter:
        """bool"""
        def __get__(self):
            cdef bool value=True
            assert self.obj.BoolParameter(ePCCalculateDiameter, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetBoolParameter(ePCCalculateDiameter, value ),"Type of setting is of different type than value"

      property boost_mult_threads:
        """bool"""
        def __get__(self):
            cdef bool value=True
            assert self.obj.BoolParameter(ePCBoostMultThreads, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetBoolParameter(ePCBoostMultThreads, value ),"Type of setting is of different type than value"

      property boost_least_sq:
        """bool"""
        def __get__(self):
            raise NotImplementedError
            cdef bool value=True
            assert self.obj.BoolParameter(ePCBoostLeastSq, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetBoolParameter(ePCBoostLeastSq, value ),"Type of setting is of different type than value"

      #2. Long
      property min_rays:
        """int"""
        def __get__(self):
            cdef long value=0
            assert self.obj.LongParameter(ePCMinRays, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetLongParameter(ePCMinRays, value ),"Type of setting is of different type than value"

      property shutter_delay: #getter and setter for shutter delay? There is already a function set group shutter delay. This function is redundant then?
        """int: in micros"""
        def __get__(self):
            cdef long value=0
            assert self.obj.LongParameter(eShutterDelay, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetLongParameter(eShutterDelay, value ),"Type of setting is of different type than value"

      property precision_packet_cap:
        """int"""
        def __get__(self):
            cdef long value=0
            assert self.obj.LongParameter(ePrecisionPacketCap, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetLongParameter(ePrecisionPacketCap, value ),"Type of setting is of different type than value"

      property object_filter_type:
        """int:  0=None, 2=Size & Roundness"""
        def __get__(self):
            cdef long value=0
            assert self.obj.LongParameter(ePCObjectFilterLevel, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetLongParameter(ePCObjectFilterLevel, value ),"Type of setting is of different type than value"

      property object_filter_minsize:
        """int"""
        def __get__(self):
            cdef long value=0
            assert self.obj.LongParameter(ePCObjectFilterMinSize, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetLongParameter(ePCObjectFilterMinSize, value ),"Type of setting is of different type than value"

      property object_filter_maxsize:
        """int"""
        def __get__(self):
            cdef long value=0
            assert self.obj.LongParameter(ePCObjectFilterMaxSize, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetLongParameter(ePCObjectFilterMaxSize, value ),"Type of setting is of different type than value"

      property object_filter_grayscale_floor:
        """int"""
        def __get__(self):
            cdef long value=0
            assert self.obj.LongParameter(ePCObjectFilterGrayscaleFloor, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetLongParameter(ePCObjectFilterGrayscaleFloor, value ),"Type of setting is of different type than value"

      property object_filter_aspect_tolerance:
        """int"""
        def __get__(self):
            cdef long value=0
            assert self.obj.LongParameter(ePCObjectFilterAspectTolerance, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetLongParameter(ePCObjectFilterAspectTolerance, value ),"Type of setting is of different type than value"

      property object_filter_object_margin:
        """int"""
        def __get__(self):
            cdef long value=0
            assert self.obj.LongParameter(ePCObjectFilterObjectMargin, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetLongParameter(ePCObjectFilterObjectMargin, value ),"Type of setting is of different type than value"

      property minimum_rank_ray_count:
        """int"""
        def __get__(self):
            cdef long value=0
            assert self.obj.LongParameter(eMinimumRankRayCount, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetLongParameter(eMinimumRankRayCount, value ),"Type of setting is of different type than value"

      property pixel_gutter:
        """int"""
        def __get__(self):
            cdef long value=0
            assert self.obj.LongParameter(ePCPixelGutter, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetLongParameter(ePCPixelGutter, value ),"Type of setting is of different type than value"

      property maximum_2D_points:
        """int"""
        def __get__(self):
            cdef long value=0
            assert self.obj.LongParameter(ePCMaximum2DPoints, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetLongParameter(ePCMaximum2DPoints, value ),"Type of setting is of different type than value"

      property calculation_time:
        """int"""
        def __get__(self):
            cdef long value=0
            assert self.obj.LongParameter(ePCCalculationTime, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetLongParameter(ePCCalculationTime, value ),"Type of setting is of different type than value"

      property thread_count:
        """int"""
        def __get__(self):
            cdef long value=0
            assert self.obj.LongParameter(ePCThreadCount, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetLongParameter(ePCThreadCount, value ),"Type of setting is of different type than value"

      property small_marker_optimization:
        """int: 0=None, 1=Fast, 2=Accurate"""
        def __get__(self):
            cdef long value=0
            assert self.obj.LongParameter(ePCSmallMarkerOptimization, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetLongParameter(ePCSmallMarkerOptimization, value ),"Type of setting is of different type than value"

      property point_cloud_engine:
        """int: 1=v1.0,  2=v2.0"""
        def __get__(self):
            cdef long value=0
            assert self.obj.LongParameter(ePointCloudEngine, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetLongParameter(ePointCloudEngine, value ),"Type of setting is of different type than value"

      property synchronizer_engine:
        """int: 1=v1.0  2=v2.0 """
        def __get__(self):
            raise NotImplementedError
            cdef long value=0
            assert self.obj.LongParameter(eSynchronizerEngine, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetLongParameter(eSynchronizerEngine, value ),"Type of setting is of different type than value"

      property marker_diameter_type:
        """int"""
        def __get__(self):
            cdef long value=0
            assert self.obj.LongParameter(eMarkerDiameterType, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetLongParameter(eMarkerDiameterType, value ),"Type of setting is of different type than value"

      property synchronizer_control:
        """int: 0=timely delivery, 1=automatic, 2=complete delivery"""
        def __get__(self):
            cdef long value=0
            assert self.obj.LongParameter(eSynchronizerControl, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetLongParameter(eSynchronizerControl, value ),"Type of setting is of different type than value"

      #3. Double
      property visible_marker_size:
        """double"""
        def __get__(self):
            cdef double value=0
            assert self.obj.DoubleParameter(eVisibleMarkerSize, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetDoubleParameter(eVisibleMarkerSize, value ),"Type of setting is of different type than value"

      property residual:
        """ double: in meters"""
        def __get__(self):
            cdef double value=0
            assert self.obj.DoubleParameter(ePCResidual, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetDoubleParameter(ePCResidual, value ),"Type of setting is of different type than value"

      property min_size:
        """double"""
        def __get__(self):
            cdef double value=0
            assert self.obj.DoubleParameter(ePCMinSize, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetDoubleParameter(ePCMinSize, value ),"Type of setting is of different type than value"

      property max_size:
        """double"""
        def __get__(self):
            cdef double value=0
            assert self.obj.DoubleParameter(ePCMaxSize, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetDoubleParameter(ePCMaxSize, value ),"Type of setting is of different type than value"

      property min_angle:
        """double"""
        def __get__(self):
            cdef double value=0
            assert self.obj.DoubleParameter(ePCMinAngle, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetDoubleParameter(ePCMinAngle, value ),"Type of setting is of different type than value"

      property min_ray_length:
        """double"""
        def __get__(self):
            cdef double value=0
            assert self.obj.DoubleParameter(ePCMinRayLength, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetDoubleParameter(ePCMinRayLength, value ),"Type of setting is of different type than value"

      property max_ray_length:
        """double"""
        def __get__(self):
            cdef double value=0
            assert self.obj.DoubleParameter(ePCMaxRayLength, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetDoubleParameter(ePCMaxRayLength, value ),"Type of setting is of different type than value"

      property reconstruct_min_x:
        """double"""
        def __get__(self):
            cdef double value=0
            assert self.obj.DoubleParameter(ePCReconstructMinX, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetDoubleParameter(ePCReconstructMinX, value ),"Type of setting is of different type than value"

      property reconstruct_max_x:
        """double"""
        def __get__(self):
            cdef double value=0
            assert self.obj.DoubleParameter(ePCReconstructMaxX, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetDoubleParameter(ePCReconstructMaxX, value ),"Type of setting is of different type than value"

      property reconstruct_min_y:
        """double"""
        def __get__(self):
            cdef double value=0
            assert self.obj.DoubleParameter(ePCReconstructMinY, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetDoubleParameter(ePCReconstructMinY, value ),"Type of setting is of different type than value"

      property reconstruct_max_y:
        """double"""
        def __get__(self):
            cdef double value=0
            assert self.obj.DoubleParameter(ePCReconstructMaxY, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetDoubleParameter(ePCReconstructMaxY, value ),"Type of setting is of different type than value"

      property reconstruct_min_z:
        """double"""
        def __get__(self):
            cdef double value=0
            assert self.obj.DoubleParameter(ePCReconstructMinZ, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetDoubleParameter(ePCReconstructMinZ, value ),"Type of setting is of different type than value"

      property reconstruct_max_z:
        """double"""
        def __get__(self):
            cdef double value=0
            assert self.obj.DoubleParameter(ePCReconstructMaxZ, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetDoubleParameter(ePCReconstructMaxZ, value ),"Type of setting is of different type than value"

      property object_filter_circularity:
        """double"""
        def __get__(self):
            cdef double value=0
            assert self.obj.DoubleParameter(ePCObjectFilterCircularity, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetDoubleParameter(ePCObjectFilterCircularity, value ),"Type of setting is of different type than value"

      property camera_overlap:
        """double"""
        def __get__(self):
            cdef double value=0
            assert self.obj.DoubleParameter(eCameraOverlap, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value):
            raise NotImplementedError
            assert self.obj.SetDoubleParameter(eCameraOverlap, value ),"Type of setting is of different type than value"

      property volume_resolution:
        """double"""
        def __get__(self):
            cdef double value=0
            assert self.obj.DoubleParameter(eVolumeResolution, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value):
            raise NotImplementedError
            assert self.obj.SetDoubleParameter(eVolumeResolution, value ),"Type of setting is of different type than value"

      property opacity:
        """double"""
        def __get__(self):
            cdef double value=0
            assert self.obj.DoubleParameter(eWireframe, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetDoubleParameter(eWireframe, value ),"Type of setting is of different type than value"

      property FOV_intensity:
        """double"""
        def __get__(self):
            cdef double value=0
            assert self.obj.DoubleParameter(eFOVIntensity, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetDoubleParameter(eFOVIntensity, value ),"Type of setting is of different type than value"

      property mask_width:
        """double"""
        def __get__(self):
            cdef double value=0
            assert self.obj.DoubleParameter(eBlockWidth, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetDoubleParameter(eBlockWidth, value ),"Type of setting is of different type than value"

      property mask_height:
        """double"""
        def __get__(self):
            cdef double value=0
            assert self.obj.DoubleParameter(eBlockHeight, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetDoubleParameter(eBlockHeight, value ),"Type of setting is of different type than value"

      property marker_diameter_force_size:
        """double"""
        def __get__(self):
            cdef double value=0
            assert self.obj.DoubleParameter(eMarkerDiameterForceSize, value),"Type of setting is of different type than value"
            return value
        def __set__(self, value): assert self.obj.SetDoubleParameter(eMarkerDiameterForceSize, value ),"Type of setting is of different type than value"

      property settings_count:
          def __get__(self):
              return eSettingsCount

