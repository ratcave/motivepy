__author__ = 'ratcave'


import motive as m
import Tkinter, tkFileDialog

root = Tkinter.Tk()
root.withdraw()
project_file_u=tkFileDialog.askopenfilename(title='Choose a project file to load: ', filetypes=[('motive projectfiles', '*.ttp')])
project_file = project_file_u.encode("ascii")
m.initialize()
m.load_project(project_file)

for i in range(0, m.camera_count()):

    # Get camera object with index i
    cam=m.Camera(i)
    cam.frame_rate = 30

    if 'Prime 13' in cam.name:
        cam.set_settings(videotype=0, exposure=480, threshold=150, intensity=0) #check if 480 corresponds to these thousands described in motive
        cam.image_gain = 8  # 8 is the maximum image gain setting
        cam.set_filter_switch(False)

    else:
        cam.set_settings(0, cam.exposure, cam.threshold, cam.intensity)

m.save_project(project_file)
m.shutdown()






