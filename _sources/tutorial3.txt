Tutorial 3: Tracking Markers and Rigid Bodies
+++++++++++++++++++++++++++++++++++++++++++++

This tutorial will guide through the process of tracking markers
and through the process of tracking rigid bodies:

    - Tracking markers in general
    - Tracking rigid bodies and their markers


Tracking markers in general
---------------------------

After the cameras have been calibrated, the 3D position data of all
markers large and bright enough can be calculated at any time.
We can extract that data in form of a tuple of nested tuples, ((x1,y1,z1),(x2,y2,z2),...,(xN,yN,zN)).::

    import motive as m

    # load a project file that includes calibration data
    m.load_project("calibrated_test.ttp")

    # update position data
    m.update()

    # get a tuple consisting of all visible 3D marker positions during the last update
    m.get_frame_markers()

Here updating the incoming camera data becomes really essential. If we had called :py:meth:`.m.get_frame_markers` before
it would have returned an empty tuple. Also :py:meth:`.m.get_frame_markers` always returns the exact same values, until :py:meth:`.m.update` is called again.

.. note:: Whenever you imported motive, you need to load a project file with calibration data to track markers


Tracking rigid bodies and their markers
---------------------------------------

After the cameras have been calibrated and one or multiple rigid bodies have been created
in the Motive GUI, if its markers are large and bright enough, a rigid body's pivot point
3D location, its rotation and the position of its markers can be calculated
at any time. To extract that data we first need to load a project file that includes rigid bodies.
We then proceed creating :py:class:`.RigidBody` objects::

    # load a project file that includes calibration data and rigid body data
    m.load_project("calibrated_and_rigid_test.ttp")

    # create a dictionary containing all rigid body objects
    rigs=m.get_rigid_bodies()

    # choose the rigid body of interest by name (just enter rigs, to see all bodies with name)
    test_rig=rigs['Arena']

Now getting a rigid body's data is as easy as calling the :py:class:`.RigidBody` object's methods.
Remember updating::

    m.update()

    # get the body's pivot point location during the last update
    test_rig.location

    # get the body's rotation in degrees and local axis
    test_rig.rotation

    # get a tuple consisting of the body's visible global 3D marker positions
    test_rig.point_cloud_markers

Of course you could always get a tuple holding all marker positions. For convenience we
also included a function that returns only the unidentified markes, namely the markers
which are not assigned to any body::

    # get a tuple consisting of all marker positions, of markers not part of any rigid body
    m.get_unident_markers()

.. note:: As of now, tracked rigid bodies can only be created via the Motive GUI


Summary:
--------

Here is a simple script including all we learned in this tutorial::


    import motive as m

    # load a project file that includes calibration data and rigid body data
    m.load_project("calibrated_and_rigid_test.ttp")

    # immediately updating is good practice
    m.update()

    # create a dictionary containing all rigid body objects
    rigs=m.get_rigid_bodies()

    # choose the rigid body of interest by name (just enter rigs, to see all bodies with name)
    test_rig=rigs['Arena']

    # get the body's pivot point location during the last update
    test_rig.location

    # get the body's rotation in degrees and local axis
    test_rig.rotation

    # get a tuple consisting of the body's visible global 3D marker positions
    test_rig.point_cloud_markers

    # get a tuple consisting of all marker positions, of markers not part of any rigid body
    m.get_unident_markers()

    # get a tuple consisting of all visible 3D marker positions
    m.get_frame_markers()

    m.save_project("test.ttp")
    m.shutdown()