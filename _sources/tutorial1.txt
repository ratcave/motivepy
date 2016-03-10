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

Saving a project file in MotivePy breaks down to one line as well::

    import motive as m

    m.save_project("test.ttp")

Which has saved the project file in the working directory.


Update and Shutdown
-------------------

As an addendum we introduce here two functions you should get
in the habit of using whenever you use MotivePy.
Once you have imported the module, the cameras will be sending
data to your PC. Upon loading a project, depending on
your project, Motive will have changed the cameras' setting
and might be able to track the 3D position of markers in the cameras'
sight. This data is not updated automatically though. To process incoming camera
data you need to call::

    m.update()

Let us assume you finished all tasks related to the cameras and data acquisition.
If you want to disconnect the cameras, simply call::

    m.shutdown()

.. note:: Always disconnect the cameras before exiting python


Summary
-------

Here is a simple script including all we learned in this tutorial::

    import motive as m

    # set camera settings, calibrate cameras, import rigid body data
    m.load_project("test.ttp")

    # process the incoming camera data
    m.update()

    # optionally save the project to file
    m.save_project("new_test.ttp")

    # disconnect the cameras
    m.shutdown()












