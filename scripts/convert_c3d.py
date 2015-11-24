__author__ = 'Nico'

import btk
import Tkinter, tkFileDialog
import os
import argparse


def convert_and_write(file):
    directory, extension = os.path.splitext(file)
    converted_file= ''.join([directory, '_vicon', extension])

    reader = btk.btkAcquisitionFileReader()
    reader.SetFilename(file)
    reader.Update()
    acq=reader.GetOutput()
    points=acq.GetPoints()
    n_points=points.GetItemNumber()

    for f in xrange(acq.GetPointFrameNumber()):
        for i in xrange(n_points):
            point=acq.GetPoint(i)
            data=point.GetValues()[f,:]
            point.SetDataSlice(f, *[1000*x for x in data])

    writer = btk.btkAcquisitionFileWriter()
    writer.SetInput(acq)
    writer.SetFilename(converted_file)
    writer.Update()


parser = argparse.ArgumentParser(description="This script converts a c3d file as exported by motive to a vicon readable c3d file.",
                                 epilog="If no arguments are given, the script first opens a window to let you choose a c3d file or a directory of c3d files to convert. \n")

parser.add_argument('-f', action='store', dest='file', default='',
                        help='Name of the file to convert.')

parser.add_argument('-F', action='store', dest='folder', default='',
                        help='Name of the folder to convert.')

args = parser.parse_args()


if args.file and not args.folder:
    convert_and_write(args.file)

elif args.folder and not args.file:
    for file in os.listdir(args.folder):
        if file.endswith(".c3d"):
            convert_and_write(file)

elif args.folder and args.file:
    raise Exception("Convert a file OR a folder")

else:
    root = Tkinter.Tk()
    root.withdraw()
    #file_u=tkFileDialog.askopenfilename(title='Choose a c3d file to convert: ', filetypes=[('motive c3d tracking files', '*.c3d')])
    tkFileDialog.askdirectory(initialdir='.') #get directory
    convert_and_write(file_u.encode("ascii"))