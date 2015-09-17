__author__ = 'ratcave'


import motive as m

user_input = raw_input("Enter complete name of project file to load:")
m.initialize()
m.load_project(user_input)

for i in range(0, m.camera_count()):

    # Get camera object with index i
    cam=m.Camera(i)
    cam.frame_rate = 30

    if 'Prime 13' in cam.name:
        cam.set_settings(videotype=0, exposure=480, threshold=150, intensity=0) #check if 480 correponds to these thousands described in motive
        cam.image_gain = 8  # 8 is the maximum image gain setting
        cam.set_filter_switch(False)

    else:
        cam.set_settings(0, m.exposure, m.threshold, m.intensity)

m.save_project(user_input)
m.shutdown()






