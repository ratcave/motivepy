__author__ = 'ratcave'


import motive as m
import Tkinter, tkFileDialog
from os import path

root = Tkinter.Tk()
root.withdraw()
project_file_u=tkFileDialog.askopenfilename(title='Choose a project file to load: ', filetypes=[('motive projectfiles', '*.ttp')])
project_file = project_file_u.encode("ascii")
m.load_project(project_file)

for cam in m.get_cams():
    cam.frame_rate = 30

    if 'Prime 13' in cam.name:
        cam.set_settings(videotype=0, exposure=33000, threshold=40, intensity=0)  #check if 480 corresponds to these thousands described in motive
        cam.image_gain = 8  # 8 is the maximum image gain setting
        cam.set_filter_switch(False)
    else:
        cam.set_settings(0, cam.exposure, cam.threshold, cam.intensity)

directory, extension = path.splitext(project_file)
saved_project_pathname= ''.join([directory, '_vislight', extension])
m.save_project(saved_project_pathname)
m.shutdown()







