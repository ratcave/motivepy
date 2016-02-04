Tutorial 2: Setting Camera Settings
+++++++++++++++++++++++++++++++++++

If you already have a Motive project file at hand which features
the camera settings you desire, simply load that project file and
you are done. But no worries if you are not in possession of such a project file yet.
MotivePy offers you the opportunity to get information about
each camera's specific settings and adapt those if need be.


1. Step: Create a Tuple of Camera Objects
-----------------------------------------

You can easily create a tuple containing a :py:class:`.Camera` object for each physically connected OptiTrack camera.
These Camera objects then offer various methods to get and set the respective camera's settings::

  import motive as m

  # create a tuple containing all camera objects
  cams=m.get_cameras()

.. note:: Whenever you import motive, each index in the camera tuple may be assigned to a different camera than last time

2. Step: Get Camera Settings
----------------------------

This is more of an optional step, which is only of practical interest for examination or verification purposes,
since you can always set camera settings to whatever value possible, independent of the original value::

   # get the zeroth indexed camera's name
   cams[0].name

   # get its frame rate
   cams[0].frame_rate

3. Step: Set Camera Settings
----------------------------

Setting settings is as straightforward as getting them::

  # set the camera's frame rate to 120 frames per second
  cams[0].frame_rate=120

  # switch the camera's filter to infrared light
  # this method has only setter properties
  cams[0].set_filter_switch(True)

To name just a couple more settings, :py:meth:`.Camera.exposure` level, :py:meth:`.Camera.image_gain` level
and the infrared illumination :py:meth:`.Camera.intensity` level.

.. note:: Every getter method has a setter method, but not the other way around.

Summary
-------

Here is a simple script including all we learned in this tutorial::

  import motive as m

  # create a tuple containing all camera objects
  cams=m.get_cameras()

  # get the zeroth indexed camera's name
  cams[0].name

  # get its frame rate
  cams[0].frame_rate

  # set its frame rate to 120 frames per second
  cams[0].frame_rate=120

  # switch its filter to infrared light
  # this method has only setter properties
  cams[0].set_filter_switch(True)

  # see first tutorial
  m.update()
  m.save_project("set_test.ttp")
  m.shutdown()





