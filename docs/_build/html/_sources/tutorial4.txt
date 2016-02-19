Tutorial 4: Getting Camera Image
++++++++++++++++++++++++++++++++

This tutorial will guide through the process of getting an image from a specific camera.
In fact MotivePy features a :py:class:`.Camera` object method which does exactly that.
But all cameras come with a variety of so called video modes, and the image format depends
on the set mode of the respective camera.

Video Modes
-----------

Depending on the type of camera there are different types of available video modes, found in Camera.____MODE class attributes.
For the camera image, you'll want to switch the camera to the GRAYSCALE_MODE (for full resolution) or MJPEG_MODE (for half resolution)::


    import motive as m
    cams = m.get_cams()
    cam = cam[0]

    # set the video mode
    cam.video_mode = m.Camera.GRAYSCALE_MODE

    m.update()

.. note:: In GRAYSCALE_MODE or MJPEG_MODE the camera can not contribute to the tracking of markers


Get Image
---------

To get the frame image, simply update the frame, and call the :py:class:`.Camera.get_frame_buffer` method on the camera you are interested in::

    img = cam.get_frame_buffer()

And that's it!  Other interesting methods are the :py:class:`Camera.frame_resolution` and :py:class:`Camera.pixel_resolution` methods, which will give you the shape of the image.


Summary
-------

Here is a simple script including all we learned in this tutorial::

    import motive as m
    cams = m.get_cams()
    cam = cam[0]

    # set the video mode
    cam.video_mode = m.Camera.GRAYSCALE_MODE

    m.update()

    # get the image
    img = cam.get_frame_buffer()

    m.save_project("test.ttp")
    m.shutdown()