__author__ = 'Vash'

# means comment from official SDK API (or Cython page) ## means comment from me

include "cnative.pxd"

def set_camera_settings(camindex, videotype, exposure, threshold, intensity):
    return TT_SetCameraSettings(camindex, videotype, exposure, threshold, intensity)

def set_camera_group(camindex, camgroupindex):
    TT_SetCameraGroup(camindex, camgroupindex)
    print "set camera group"