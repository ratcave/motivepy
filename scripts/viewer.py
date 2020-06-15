__author__ = 'nico'

## FOR DEBUGGING
#import pdb
#pdb.set_trace()

import Tkinter, tkFileDialog
import argparse
import motive
from motive import utils


if __name__ == '__main__':

   # Get command line inputs
    parser = argparse.ArgumentParser(description="This is the motive viewer script. It displays the unidentified markers and the rigid bodies in real time.",
                                     epilog="If no arguments are given, the script first opens a window to let you search for a project file to load. \n")

    parser.add_argument('-l', action='store_true', dest='last_profile', default=False,
                        help='If this flag is set, the last profile file is loaded.')

    parser.add_argument('-p', action='store', dest='profile_filename', default='',
                        help='Name of the profile file to load.')

    parser.add_argument('-love', action='store', dest='love', default='',
                        help='Plot a heart instead.')

    args = parser.parse_args()

    # Get Project
    if args.last_profile:
        if args.profile_filename is True:
            raise Warning("Load last profile file OR name a profile file to load.")
        profile_file=utils.backup_profile_filename

    elif args.profile_filename:
        if args.last_profile is True:
            raise Warning("Load last profile file OR name a profile file to load.")
        profile_file=args.profile_filename

    else:
        root = Tkinter.Tk()
        root.withdraw()
        profile_file_u=tkFileDialog.askopenfilename(title='Choose a profile file to load: ', filetypes=[('motive profilefiles', '*.motive')])
        profile_file = profile_file_u.encode("ascii")

    # Load Motive Project
    motive.load_profile(profile_file)

    if args.love:
        import matplotlib.pyplot as plt
        import numpy as np
        t=np.arange(0,2*np.pi, 0.1)
        x, y =   16*np.sin(t)**3, 13*np.cos(t)-5*np.cos(2*t)-2*np.cos(3*t)-np.cos(4*t)
        plt.plot(x,y)
        plt.show()

   # Display viewer
    motive.show_viewer()