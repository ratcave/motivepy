__author__ = 'nico'

import motive
import Tkinter, tkFileDialog

#import pdb
#pdb.set_trace()

if __name__ == '__main__':

    # Get Project
    root = Tkinter.Tk()
    root.withdraw()
    project_file_u=tkFileDialog.askopenfilename(title='Choose a project file to load: ', filetypes=[('motive projectfiles', '*.ttp')])
    project_file = project_file_u.encode("ascii")

    # Load Motive Project
    motive.load_project(project_file)

    # Display viewer
    motive.show_viewer()