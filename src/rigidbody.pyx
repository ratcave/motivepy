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




#FUNCTIONS
def get_unident_markers():
     """returns a tuple of all markers which are not in rigid Bodies"""
     markers=tuple((TT_FrameMarkerX(i), TT_FrameMarkerY(i), TT_FrameMarkerZ(i)) for i in xrange(TT_FrameMarkerCount()))
     rigs=get_rigid_bodies()
     imarkers=[]
     unimarkers=[]
     for i in range (1,TT_RigidBodyCount()):
        for ik in rigs[i].point_cloud_markers:
            imarkers.append(ik)
     for k in markers:
        if k not in imarkers:
            unimarkers.append(k)
     return tuple(unimarkers)

def get_rigid_bodies():
    """Initiate all loaded rigid bodies as python objects, where rigid body #k is [k-1]"""
    return tuple(RigidBody(rigidIndex) for rigidIndex in xrange(TT_RigidBodyCount()))

@check_npresult
def create_rigid_body(str name, markerList):
     """
     The marker list is expected to contain a list of marker coordinates in the order:  x1,y1,z1,x2,y2,z2,...xN,yN,zN.
     For some reason a rigid body created via this function in python, is not tracked,
     and seemingly cannot be tracked.
     """
     raise NotImplementedError()
     cdef float markerListp[1000]
     markerCount=len(markerList)
     assert markerCount<=1000, "Due to need of const C array size, markerList max items=1000. \n Please resize const in native.pyx"
     for i in range(0,len(markerList)):
         markerListp[3*i]=markerList[i][0]
         markerListp[3*i+1]=markerList[i][1]
         markerListp[3*i+2]=markerList[i][2]

     rigidIndexplus1=TT_RigidBodyCount()+1
     return TT_CreateRigidBody(name, rigidIndexplus1 , markerCount, markerListp)

@check_npresult
def remove_rigid_body(int rigidIndex):
    """Remove single rigid body"""
    return TT_RemoveRigidBody(rigidIndex)

def clear_rigid_body_list():
    """Clear all rigid bodies"""
    TT_ClearRigidBodyList()


#CLASS
class RigidBody(object):
    def __init__(self, rigidIndex):
        """Returns a RigidBody Motive API object."""
        assert 0<=rigidIndex<TT_RigidBodyCount(), "There Are Only {0} Rigid Bodies".format(TT_RigidBodyCount())
        self.index=rigidIndex

    def __str__(self):
        return "Rigid Body Object: {0}".format(self.name)

    def __repr__(self):
        return self.__str__()

    @property
    def name(self):
        """Returns RigidBody Name"""
        return "{0}".format(TT_RigidBodyName(self.index))

    @property
    def user_data(self):
        """
        Get RigidBodies User Data
        """
        return "Rigid body ID: {0}".format(TT_RigidBodyUserData(self.index))

    @user_data.setter
    def user_data(self,value):
        TT_SetRigidBodyUserData(self.index,value)

    @property
    def enabled(self):
        """Get tracking (bool)"""
        return TT_RigidBodyEnabled(self.index)

    @enabled.setter
    def enabled(self, value):
        TT_SetRigidBodyEnabled(self.index, value)

    @property
    def is_tracked(self):
        """
        Is rigid body currently tracked
        """
        return TT_IsRigidBodyTracked(self.index)

    def get_all_spatial_data(self):
        """Returns dict: {'location': (x, y, z), 'rotation':(yaw, pitch, roll), 'rotation_quats':(qx, qy, qz, qw)}.
        This is done in a single C function call, so it's really fast if you want all the data."""
        cdef float x = 0., y = 0., z = 0., qx = 0., qy = 0., qz = 0., qw = 0., yaw = 0., pitch = 0., roll = 0.
        TT_RigidBodyLocation(self.index,  &x, &y, &z,  &qx, &qy, &qz, &qw, &yaw, &pitch, &roll)
        return {'location': (x, y, z), 'rotation': (yaw, pitch, roll),'rotation_quats': {qx, qy, qz, qw}}

    @property
    def location(self):
        """(x, y, z) position."""
        return self.get_all_spatial_data()['location']

    @property
    def rotation(self):
        """(yaw, pitch, roll) rotation"""
        return self.get_all_spatial_data()['rotation']

    @property
    def rotation_quats(self):
        """(qx, qy, qz, qw) quaternion rotation."""
        return self.get_all_spatial_data()['rotation_quats']

    @property
    def markers(self):
        """Get list of rigid body marker position"""
        markers=[]
        cdef float x = 0, y = 0, z = 0
        for i in range(0, TT_RigidBodyMarkerCount(self.index)):
            TT_RigidBodyMarker(self.index, i, &x, &y, &z)
            markers.append([x, y, z])
        return markers

    @property
    def point_cloud_markers(self):
        """Gets tuple of point cloud markers."""            #Tuples for the location of each marker is good. But all locations together seems more feasible for list!
        markers = []
        cdef int markerIndex
        cdef bool tracked = True
        cdef float x = 0, y = 0, z = 0  # Says it works at http://docs.cython.org/src/userguide/pyrex_differences.html
        for markerIndex in xrange(TT_RigidBodyMarkerCount(self.index)):
            # Get marker position
            TT_RigidBodyPointCloudMarker(self.index, markerIndex, tracked, x, y, z)
            # Add the marker if one was found (tracked was True). Else, substitute by rigid body position
            marker = (x, y, z) if tracked else self.location
            markers.append(marker)

        return tuple(markers)

    @check_npresult
    def translate_pivot(self, float x, float y, float z):
        """
        Rigid Body Pivot-Point Translation: Sets a translation offset for the centroid of the rigid body.
        Reported values for the location of the rigid body, as well as the 3D visualization, will be shifted
        by the amount provided in the fields on either the X, Y, or Z axis. Values are entered in meters.
        """
        return TT_RigidBodyTranslatePivot(self.index, x, y, z)

    def reset_orientation(self):
        """Reset orientation to match the current tracked orientation of the rigid body."""
        TT_RigidBodyResetOrientation(self.index)