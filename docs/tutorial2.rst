Tutorial 2: Track Markers and Rigid Bodies
++++++++++++++++++++++++++++++++++++++++++

This tutorial will guide through the process of tracking markers
and through the process of tracking rigid bodies:

    - Track markers in general
    - Track rigid bodies and their markers

Track markers in general
------------------------

After the cameras have been calibrated, the 3D position data of all
markers large and bright enough can be calculated at any time.
We can extract that data in form of a tuple of nested tuples, ((x1,y1,z1),(x2,y2,z2),...,(xN,yN,zN))::

    import motive as m

    #load a project file that includes a calibration file
    m.load_project("calibrated_test.ttp")

    #update position data
    m.update()

    #get a tuple consisting of all visible 3D marker positions during the last update
    m.get_frame_markers()

    .
    .
    .

    #make sure to shutdown the camera connections before exiting
    m.shutdown()


Track rigid bodies and their markers
------------------------------------

After the cameras have been calibrated and one or multiple rigid bodies have been created
in the Motive GUI, if its markers are large and bright enough, a rigid body's pivot point
3D location, its rotation and the position of its markers can be calculated
at any time. To extract that data we create :py:class:`.RigidBody` objects and use their respective methods::

    import motive as m

    #load a project file that includes a calibration file and a rigid body file
    m.load_project("calibrated_and_rigid_test.ttp")

    #create a tuple containing all rigid body objects
    rigs=m.get_rigid_bodies()

    #get the rigid body's name
    rigs[0].name

    #update position data
    m.update()

    #get the body's pivot point location during the last update
    rigs[0].location

    #get the body's rotation in degrees and local axis
    rigs[0].rotation

    #get a tuple consisting of the body's visible global 3D marker positions
    rigs[0].point_cloud_markers

    #get a tuple consisting of all marker positions, of markers not part of any rigid body
    m.get_unident_markers()

    .
    .
    .

    #make sure to shutdown the camera connections before exiting
    m.shutdown()

.. note:: As of now, tracked rigid bodies can only be created via the Motive GUI