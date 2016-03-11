Tutorial 5: Setting Point Cloud Settings
++++++++++++++++++++++++++++++++++++++++

The Motive API features the possibility to get and set
Point Cloud Settings in a project file.
You can set the settings you like and save those
in a project file, which you can then load in
another Motive Session, in the API or the GUI.
You can also get the settings from a project file
which has been created in the GUI or the API in an earlier
session.


1. Step: Create a Settings Objects
----------------------------------

In order to set or get point cloud settings you first need to create a :py:class:`.CameraGroupPointCloudSettings` object.
This object will eventually store the values for all settings::

  import motive as m

  # create the point cloud settings objects
  settings=m.CameraGroupPointCloudSettings()


2. Step: Get Point Cloud Settings
----------------------------

This is more of an optional step, which is only of practical interest for examination or verification purposes,
since you can always set point cloud settings to whatever value possible, independent of the original value::

   # load the project file which features the settings of interest
   m.load_project("test.ttp")

   # writes the settings of the file for camera group 1 (default group) in the settings object
   m.get_camera_group_point_cloud_settings(0,settings)

   # get the filter type
   settings.object_filter_type

.. warning:: This has only been tested for the default camera group

3. Step: Set Point Cloud Settings
----------------------------

Setting settings is as straightforward as getting them::

  # set the filter type to size and roundness in the settings object
  settings.object_filter_type=2

  # apply the new settings to camera group 1 point cloud
  m.set_camera_group_point_cloud_settings(0,settings)


To name just a couple more settings, :py:meth:`.CameraGroupPointCloudSettings.calculation_time`, :py:meth:`.CameraGroupPointCloudSettings.point_cloud_engine`
and the :py:meth:`.CameraGroupPointCloudSettings.residual` size in meters.


Summary
-------

Here is a simple script including all we learned in this tutorial::

  import motive as m

  # create the point cloud settings objects
  settings=m.CameraGroupPointCloudSettings()

  # load the project file which features the settings of interest
   m.load_project("test.ttp")

  # writes the settings of the file for camera group 1 (default group) in the settings object
  m.get_camera_group_point_cloud_settings(0,settings)

  # get the filter type
  settings.object_filter_type

  # set the filter type to size and roundness in the settings object
  settings.object_filter_type=2

  # apply the new settings to camera group 1 point cloud
  m.set_camera_group_point_cloud_settings(0,settings)

  # see first tutorial
  m.update()
  m.save_project("set_test.ttp")
  m.shutdown()





