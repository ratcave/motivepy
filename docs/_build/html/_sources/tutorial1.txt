Tutorial 1: Setting Camera Settings
+++++++++++++++++++++++++++++++++++

This tutorial will guide through three processes of which each suffices to set a specific camera's settings:

  1.  Process:  Load a Motive project file
  2.  Process:  Create a :py:class:`.Camera` object and use its methods
  3.  Process:  Load a Motive project file and create a :py:class:`.Camera` object to use its methods

As the third process is combining the first and the second, we will employ it also as a point for summary.


1. Process: Load a Motive project file
--------------------------------------

The simplest way to set OptiTrack camera settings is to load a Motive project file.
Alas, this is a bit of a chicken and egg situation. In order to load a project file, this
file needed to be created first. There are two possibilities to create such a file:

  - Open the Motive GUI, set each setting manually and save the project
  - Set camera settings through API as described below (2. Process) and save the project

Once we have a project (.ttp) file which features the camera settings we need, we can
then load this project and voila, our cameras will be set as desired::

  import motive as m

  #project file must have .ttp extension
  m.load_project("test.ttp")

  .
  .
  .

  #make sure to shutdown the connection with the cameras before exiting
  m.shutdown()


2. Process: Create a Camera object and use its methods
------------------------------------------------------

If you do not have a project file with the desired camera settings at hand, the fastet way to
set those settings is by creating :py:class:`.Camera` objects and using their respective methods::

  import motive as m

  #create a tuple containing all camera objects
  cams=m.get_cameras()

  #get the first camera's name
  cams[0].name

  #get its frame rate, set a new frame rate and get the newly set frame rate
  cams[0].frame_rate
  cams[0].frame_rate=120
  cams[0].frame_rate

  #switch the camera's filter to infrared light
  #this method has only setter properties
  cams[0].set_filter_switch(True)

  #to follow up on the 1. process, this is how you save the settings in a project file
  m.save_project("test_set.ttp")

  .
  .
  .

  #make sure to shutdown the connection with the cameras before exiting
  m.shutdown()


To name just a couple more settings, :py:meth:`.Camera.exposure` level, :py:meth:`.Camera.image_gain` level
and the infrared illumination :py:meth:`.Camera.intensity` level.

.. note:: Whenever you import motive, the camera index in the tuple may denote a different camera

3. Process: Load a Motive project file and create a Camera object to use its methods
------------------------------------------------------------------------------------

Whenever you have a project file which features some settings you desire but others
you need to adapt, it might make sense to first load that project (1. Process) and then
only set the settings you need to adapt (2.Process). Let us suppose for example you
are happy with the frame rate for camera X in test_set.ttp but you want to switch
the camera's filter to visible light::

  import motive as m

  m.load_project("test_set.ttp")

  cams=m.get_cameras()

  #make sure the zeroth index still denotes camera X!
  cams[0].name

  cams[0].set_filter_switch(False)

  #if you want to save the new settings to a project file
  m.save_project("newtest_set.ttp")

  .
  .
  .

  #make sure to shutdown the connection with the cameras before exiting
  m.shutdown()


.. note::  Whenever you load a project file, this overwrites all camera settings





