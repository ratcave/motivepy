"""Motive Rigid Body Module

This module features all functionality related to rigid body properties.
It is basically made up of one large rigid body class.
The basic function in this module is get_rigid_bodies().

Examples::

        >>>get_rigid_bodies()
        (Rigid Body Object: Arena, Rigid Body Object: Head)
        >>>rigs=get_rigid_bodies()
        >>>rigs[0].name
        Arena

"""

include "cnative.pxd"

from motive import utils

#FUNCTIONS
def get_unident_markers():
     """Returns a tuple containing all tuples of 3D marker positions not part of rigid bodies"""
     markers=tuple((TT_FrameMarkerX(i), TT_FrameMarkerY(i), TT_FrameMarkerZ(i)) for i in xrange(TT_FrameMarkerCount()))
     rigs=get_rigid_bodies().values()
     imarkers=[]
     unimarkers=[]
     for i in range (0,TT_RigidBodyCount()):
        for ik in rigs[i].point_cloud_markers:
            imarkers.append(ik)
     for k in markers:
        if k not in imarkers:
            unimarkers.append(k)
     return tuple(unimarkers)


def get_rigid_bodies():
    """Returns a tuple containing all rigid bodies

    Note:
        First load a project file or a rigid body file.
        For more information on this see native.pyx.

    Returns:
        Tuple of rigid body objects
    """
    return {RigidBody(idx).name: RigidBody(idx) for idx in xrange(TT_RigidBodyCount())}


# @utils.decorators.check_npresult
# def create_rigid_body(str name, markerList):
#      """The marker list is expected to contain a list of marker coordinates in the order:  x1,y1,z1,x2,y2,z2,...xN,yN,zN.
#      For some reason a rigid body created via this function in python, is not tracked,
#      and seemingly cannot be tracked.
#      We therefore decided to let it raise a not implemented error.
#      """
#      raise NotImplementedError()
#      markerCount=len(markerList)
#      cdef float * markerListp=<float *> malloc(markerCount*sizeof(float))
#      for i in xrange(len(markerList)):
#          markerListp[3*i]=markerList[i][0]
#          markerListp[3*i+1]=markerList[i][1]
#          markerListp[3*i+2]=markerList[i][2]
#
#      rigidIndexplus1=TT_RigidBodyCount()+1
#      return TT_CreateRigidBody(name, rigidIndexplus1 , markerCount, markerListp)


@utils.decorators.check_npresult
def remove_rigid_body(int rigidIndex):
    """Removes a rigid body object

    Args:
        rigidIndex(int): The index of the rigid body
    """
    return TT_RemoveRigidBody(rigidIndex)


def clear_rigid_body_list():
    """Removes all rigid bodies"""
    TT_ClearRigidBodyList()


#CLASS
class RigidBody(object):
    def __init__(self, rigidIndex):
        """Returns a rigid body object

        Args:
            rigidIndex (int): The index of the rigid body to be returned
        Raises:
            AssertionError: If the index is larger than the number of rigid bodies
        """
        assert 0<=rigidIndex<TT_RigidBodyCount(), "There Are Only {0} Rigid Bodies".format(TT_RigidBodyCount())
        self.index=rigidIndex

    def __str__(self):
        return "Rigid Body Object: {0}".format(self.name)

    def __repr__(self):
        return self.__str__()

    @property
    def name(self):
        """str: Rigid body name"""
        return "{0}".format(TT_RigidBodyName(self.index))

    @property
    def user_data(self):
        """int: Rigid body user data"""
        return "Rigid body ID: {0}".format(TT_RigidBodyUserData(self.index))

    @user_data.setter
    def user_data(self,value):
        TT_SetRigidBodyUserData(self.index,value)

    @property
    def enabled(self):
        """bool: Rigid body tracking enabled"""
        return TT_RigidBodyEnabled(self.index)

    @enabled.setter
    def enabled(self, value):
        TT_SetRigidBodyEnabled(self.index, value)

    @property
    def is_tracked(self):
        """bool: Rigid body is tracked"""
        return TT_IsRigidBodyTracked(self.index)

    def get_all_spatial_data(self):
        """Returns spatial data of the rigid body

        Returns:
            Dictionary, maps identifying strings to tuples of data::

            {
                'location': (x, y, z),
                'rotation': (yaw, pitch, roll),
                'rotation_quats': (qx, qy, qz, qw)
            }

        """
        cdef float x = 0., y = 0., z = 0., qx = 0., qy = 0., qz = 0., qw = 0., yaw = 0., pitch = 0., roll = 0.
        TT_RigidBodyLocation(self.index,  &x, &y, &z,  &qx, &qy, &qz, &qw, &yaw, &pitch, &roll)
        return {'location': (x, y, z), 'rotation': (yaw, pitch, roll),'rotation_quats': {qx, qy, qz, qw}}

    @property
    def location(self):
        """Tuple[float]: Rigid body's (x, y, z) position"""
        return self.get_all_spatial_data()['location']

    @property
    def rotation(self):
        """Tuple[float]: Rigid body's (yaw, pitch, roll) rotation, in degrees and in local axis"""
        return self.get_all_spatial_data()['rotation']

    @property
    def rotation_global(self):
        """Tuple[float]: Rigid body's (x, y, z) rotation, in degrees and in world axis"""
        rotation = self.get_all_spatial_data()['rotation']
        return tuple(rotation[el] for el in [1, 0, 2])

    @property
    def rotation_quats(self):
        """Tuple[float]: Rigid body's (qx, qy, qz, qw) quaternion rotation"""
        return self.get_all_spatial_data()['rotation_quats']

    @property
    def markers(self):
        """Tuple[float]: Rigid body's 3D marker positions"""
        markers=[]
        cdef float x = 0, y = 0, z = 0
        for i in range(0, TT_RigidBodyMarkerCount(self.index)):
            TT_RigidBodyMarker(self.index, i, &x, &y, &z)
            markers.append((x, y, z))
        return tuple(markers)

    @property
    def point_cloud_markers(self):
        """Tuple[float]:  Rigid body's 3D point cloud marker positions."""
        markers = []
        cdef int markerIndex
        cdef bool tracked = True
        cdef float x = 0, y = 0, z = 0
        for markerIndex in xrange(TT_RigidBodyMarkerCount(self.index)):
            # Get marker position
            TT_RigidBodyPointCloudMarker(self.index, markerIndex, tracked, x, y, z)
            # Add the marker if one was found (tracked was True). Else, substitute by rigid body position
            marker = (x, y, z) if tracked else self.location
            markers.append(marker)

        return tuple(markers)               #Tuples for the location of each marker is good. But all locations together seems more feasible for list!

    @utils.decorators.check_npresult
    def translate_pivot(self, float x, float y, float z):
        """Sets a translation offset for the centroid of the rigid body

        Reported values for the location of the rigid body, as well as the 3D visualization, will be shifted
        by the amount provided in the fields on either the X, Y, or Z axis. Values are entered in meters.

        Args:
            x(float): shift of rigid body position in X direction in meters
            y(float): shift of rigid body position in Y direction in meters
            z(float): shift of rigid body position in Z direction in meters
        """
        return TT_RigidBodyTranslatePivot(self.index, x, y, z)

    def reset_orientation(self):
        """Resets the rigid body's orientation to match its current tracked orientation"""
        TT_RigidBodyResetOrientation(self.index)