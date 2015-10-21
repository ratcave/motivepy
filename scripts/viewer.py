__author__ = 'nico'

## FOR DEBUGGING
#import pdb
#pdb.set_trace()

import motive
from motive import utils
import Tkinter, tkFileDialog
import argparse


if __name__ == '__main__':

   # Get command line inputs
    parser = argparse.ArgumentParser(description="This is the motive viewer script.  It shows the unidentified markers and the rigid bodies in real time.")

    parser.add_argument('-l', action='store', dest='last_project', default=False,
                        help='If this flag is set, the last project file is loaded.')

    parser.add_argument('-p', action='store', dest='project_filename', default='',
                        help='Name of the project file to load.')

    args = parser.parse_args()

    # Get Project
    if args.last_project:
        if args.project_filename is True:
            raise Warning("Load last project file OR name a project file to load.")
        project_file=utils.backup_project_filename

    elif args.project_filename:
        if args.last_project is True:
            raise Warning("Load last project file OR name a project file to load.")
        project_file=args.project_filename

    else:
        root = Tkinter.Tk()
        root.withdraw()
        project_file_u=tkFileDialog.askopenfilename(title='Choose a project file to load: ', filetypes=[('motive projectfiles', '*.ttp')])
        project_file = project_file_u.encode("ascii")

    # Load Motive Project
    motive.load_project(project_file)

   # Display viewer
    motive.show_viewer()