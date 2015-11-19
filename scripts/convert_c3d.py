__author__ = 'Nico'

import btk
import Tkinter, tkFileDialog
from os import path

root = Tkinter.Tk()
root.withdraw()
file_u=tkFileDialog.askopenfilename(title='Choose a c3d file to convert: ', filetypes=[('motive c3d tracking files', '*.c3d')])
file = file_u.encode("ascii")
directory, extension = path.splitext(file)
converted_file= ''.join([directory, '_vicon', extension])

reader = btk.btkAcquisitionFileReader()
reader.SetFilename(file)            #TODO: be able to change filename when running script
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
writer.SetFilename(converted_file)     #TODO: see above
writer.Update()