__author__ = 'Nico'

import btk


reader = btk.btkAcquisitionFileReader()
reader.SetFilename("moving.c3d")            #TODO: be able to change filename when running script
reader.Update()
acq=reader.GetOutput()
points=acq.GetPoints()
n_points=points.GetItemNumber()

for f in xrange(acq.GetPointFrameNumber()):
    for i in xrange(n_points):
        point=acq.GetPoint(i)
        data=point.GetValues()[f,:]
        data=[1000*x for x in data]
        point.SetDataSlice(f, *data)

writer = btk.btkAcquisitionFileWriter()
writer.SetInput(acq)
writer.SetFilename("moving_script_btk.c3d")     #TODO: see above
writer.Update()