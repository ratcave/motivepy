Tutorial 1: Saving and Loading Motive Project Files
+++++++++++++++++++++++++++++++++++++++++++++++++++

When working with the Motive GUI or MotivePy, the most important
file is the project (.ttp) file. It always contains settings for
every camera which are automatically set upon loading the file.
In addition it can contain calibration data, which is essential for
3D tracking of marker positions. Furthermore it can contain rigid
body data, which in turn is essential for 3D tracking of the rigid
body's pivot position, its rotation and its associated marker positions.
All this data is available, once a project file has been loaded.


Loading a Motive Project File
-----------------------------

Loading a project file in MotivePy is basically a one liner::

    import motive as m

    # the project file needs a .ttp extension
    m.load_project("test.ttp")

Now you have loaded a project. In the most basic case, when your project
did not contain calibration- or rigid body data, it will still contain
camera settings. These are now set.

.. note::  Whenever you load a project file, this overwrites all camera settings


Saving a Motive Project File
----------------------------

Saving a project file in MotivePy breaks down to a one liner as well::

    import motive as m

    m.save_project("test.ttp")

Which has saved the project file in the working directory.


Update
-------------------

Once you have imported the module, the cameras will be sending
data to your PC. Once you have loaded a project, depending on
your project, Motive will have changed the cameras' setting
and might be able to track the 3D position of markers in the cameras'
sight. This data is not updated automatically though. To process
incoming camera data 










