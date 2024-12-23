"""Motive Rigid Body Module

This module features all functionality related to rigid body properties.
It is basically made up of one large rigid body class.
The basic function in this module is get_rigid_bodies().

Examples::

        >>>get_rigid_bodies()
        {'Arena': Rigid Body Object: Arena, 'Head': Rigid Body Object: Head}
        >>>rigs=get_rigid_bodies()
        >>>test_rig=rigs['Arena']
        >>>test_rig.name
        Arena

"""
from __future__ import absolute_import

include "cnative.pxd"

from cpython cimport array
cdef extern from *:
    wchar_t* PyUnicode_AsWideCharString(object, Py_ssize_t *size)
    object PyUnicode_FromWideChar(const wchar_t *w, Py_ssize_t size)
import array
from . import native
from .decorators import convert_string_output
from libc.stdlib cimport malloc, free
from collections import namedtuple
import warnings
import _transformations as trans
import numpy as np
import itertools

Quaternion = namedtuple('Quaternion', 'x y z w')
EulerRotation = namedtuple('EulerRotation', 'yaw pitch roll')
Location = namedtuple('Location', 'x y z')

#FUNCTIONS
def get_unident_markers():
    """Returns a tuple containing all tuples of 3D marker positions not part of rigid bodies"""
    cdef float x=0, y=0, z=0
    cdef list coords = []
    for i in range(MarkerCount()):
        # Call the function; it returns True on success (assumes success for all)
        MarkerXYZ(i, x, y, z)
        coords.append((x, y, z))

    markers=tuple(coords)
#    markers=tuple((FrameMarkerX(i), FrameMarkerY(i), FrameMarkerZ(i)) for i in xrange(FrameMarkerCount()))
    rigs=get_rigid_bodies().values()
    imarkers=[]
    unimarkers=[]
    for name, body in get_rigid_bodies().items():
        imarkers.extend(body.point_cloud_markers)
    for k in markers:
        if k not in imarkers:
            unimarkers.append(k)
    return tuple(unimarkers)

def get_all_rigid_bodies_markers():
    cdef float x=0, y=0, z=0
    cdef list coords = []
    for i in range(MarkerCount()):
        # Call the function; it returns True on success (assumes success for all)
        MarkerXYZ(i, x, y, z)
        coords.append((x, y, z))

    markers=[coords]
    return markers

def get_rigid_bodies():
    """Returns a dictionary containing all rigid bodies."""
    return RigidBody.get_all()


def create_rigid_body(str name, markerList):
    """Creates a new rigid body

        Args:
        name(str): Name of the rigid body to be created
        markerList(List[float]): A list of marker coordinates in the order:  [[x1,y1,z1], [x2,y2,z2], ..., [xN,yN,zN]]
        Note:
        For some reason a rigid body created via this API function is not tracked by Motive.
    """
    cdef array.array markerList_array = array.array('f', itertools.chain(*markerList))
    cdef Py_ssize_t length
    return CreateRigidBody(PyUnicode_AsWideCharString(name, &length), RigidBody.count() + 1, len(markerList), markerList_array.data.as_floats)


def remove_rigid_body(int rigidIndex):
    """Removes a rigid body object

    Args:
        rigidIndex(int): The index of the rigid body
    """
    return native.check_npresult(RemoveRigidBody)(rigidIndex)


#CLASS
class RigidBody(object):
    def __init__(self, index):
        """Returns a rigid body object

        Args:
            index (int): The index of the rigid body to be returned
        Raises:
            AssertionError: If the index is larger than the number of rigid bodies
        """
        if index < 0:
            raise ValueError("Index must be Positive")
        if index >= self.count():
            raise ValueError("Index {} too High: Only {} Rigid Bodies detected.".format(index, RigidBody.count()))
        self.index = index

    def __str__(self):
        fmt = '{cls}(index={index}, name="{name}")'
        return fmt.format(cls=self.__class__.__name__, index=self.index, name=self.name)

    def __repr__(self):
        return self.__str__()

    @staticmethod
    def count():
        return RigidBodyCount()

    @classmethod
    def get_all(cls):
        """Returns dict of all RigidBodies."""
        return {cls(idx).name: cls(idx) for idx in range(cls.count())}

    @property
    def name(self):
        """str: Rigid body name"""
        cdef wchar_t name[256]
        RigidBodyName(self.index, name, 256)
        return PyUnicode_FromWideChar(name, -1)

    @property
    def enabled(self):
        """bool: Rigid body tracking enabled"""
        return RigidBodyEnabled(self.index)

    @enabled.setter
    def enabled(self, value):
        SetRigidBodyEnabled(self.index, value)

    @property
    def is_tracked(self):
        """bool: Rigid body is tracked"""
        return IsRigidBodyTracked(self.index)

    def get_all_spatial_data(self):
        """Returns spatial data of the rigid body

        Returns::

            {
                'location': (x, y, z),
                'rotation': (yaw, pitch, roll),
                'rotation_quats': (qx, qy, qz, qw)
            }

        """
        cdef float x = 0., y = 0., z = 0., qx = 0., qy = 0., qz = 0., qw = 0., yaw = 0., pitch = 0., roll = 0.
        RigidBodyTransform(self.index,  &x, &y, &z,  &qx, &qy, &qz, &qw, &yaw, &pitch, &roll)
        return {'location': Location(x, y, z), 'rotation': EulerRotation(yaw, pitch, roll),'rotation_quats': Quaternion(qx, qy, qz, qw)}

    @property
    def location(self):
        """Tuple[float]: Rigid body's (x, y, z) position"""
        return self.get_all_spatial_data()['location']

    @location.setter
    def location(self, coords):
        q = self.rotation_quats
        rot = (q.w, q.x, q.y, q.z)
        rot_matrix = trans.quaternion_matrix(rot)[:3, :3]
        coords_offset = np.array(coords) - np.array(self.location)
        translation_local = np.dot(np.linalg.pinv(rot_matrix), coords_offset)
        self.translate_pivot(*translation_local)
        native.update()  # important, to prevent false answers from multiple location-setting calls


    @property
    def rotation(self):
        """Tuple[float]: Rigid body's (yaw, pitch, roll) rotation, in degrees and in local axis"""
        return self.get_all_spatial_data()['rotation']

    @property
    def rotation_quats(self):
        """Tuple[float]: Rigid body's (qx, qy, qz, qw) quaternion rotation"""
        return self.get_all_spatial_data()['rotation_quats']

    @property
    def markers(self):
        """Tuple[float]: Rigid body's local 3D marker positions"""
        markers=[]
        cdef float x = 0, y = 0, z = 0
        for i in range(0, RigidBodyMarkerCount(self.index)):
            RigidBodyMarker(self.index, i, &x, &y, &z)
            markers.append((x, y, z))
        return tuple(markers)


    #@native.check_npresult
    def translate_pivot(self, float x, float y, float z):
        """Sets a translation offset for the centroid of the rigid body

        Reported values for the location of the rigid body, as well as the 3D visualization, will be shifted
        by the amount provided in the fields on either the X, Y, or Z axis. Values are entered in meters.

        Args:
            x(float): shift of rigid body position in X direction in meters
            y(float): shift of rigid body position in Y direction in meters
            z(float): shift of rigid body position in Z direction in meters

        Note: To set a new World position for the Rigidbody, simply set a new location: (ex: Rigidbody.location = 1, 2, 3)
        """

        if native.get_build_number() > 26069:
            warnings.warn('Negating X axis for pivot translation to counter Motive bug.  Not tested for the current build version--use with care.')

        x *= -1.
        return native.check_npresult(RigidBodyTranslatePivot)(self.index, x, y, z)

    def reset_pivot_offset(self):
        self.location = np.mean(self.point_cloud_markers, axis=0)

    def reset_orientation(self):
        """Resets the rigid body's orientation to match its current tracked orientation"""
        RigidBodyResetOrientation(self.index)