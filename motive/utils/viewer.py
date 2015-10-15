__author__ = 'nico'

def viewer():
    # Get Project
    root = Tkinter.Tk()
    root.withdraw()
    project_file_u=tkFileDialog.askopenfilename(title='Choose a project file to load: ', filetypes=[('motive projectfiles', '*.ttp')])
    project_file = project_file_u.encode("ascii")

    # Load Motive Project
    m.load_project(project_file)
    m.update()

    # Create Qt Window
    app = QtGui.QApplication([])            #create the graphing application
    w = gl.GLViewWidget()                   #create widget
    w.opts['distance'] = 4                  #start distance from where one looks at the plot
    w.setFixedSize(1100, 800)
    y_up_rotation = (90, 1, 0, 0)

    # Initialize/Set unident_marker plot
    unident_markers = gl.GLScatterPlotItem(pos=array([[0,0,0]]), color=(204/255, 1, 1, 0.8), size=6)  #pos needs array because it needs shape
    unident_markers.rotate(*y_up_rotation)
    w.addItem(unident_markers)

    # Initialize/Set rigid_marker plot for every rigid body (except arena!)
    rigs=m.get_rigid_bodies()

    # Assign plot color to the rigid bodies
    color_dict = {'Red': (1., 0., 0.),
                  'Green': (0., 1., 0.),
                  'Yellow': (1., 1., 0.),
                  'Mag.': (1., 0., 1.),
                  'Orange': (1., .4, 0.)}
    color_cycler = itertools.cycle(color_dict)
    for rig, color_name in zip(rigs, color_cycler):
        rig.color_name = color_name
        rig.color_val = color_dict[color_name] + (1.0,)

    # Initialize the Rigid Body Scatterplot
    for rig in rigs:
        scat = gl.GLScatterPlotItem(pos=array([[0,0,0]]), color=rig.color_val, size=8)  #pos needs array because it needs shape
        scat.rotate(*y_up_rotation)
        w.addItem(scat)

    # Make floor rectangle
    grid_points = linspace(-1, 1, 200)
    points_2d =  array(list(itertools.product(grid_points, grid_points)))
    points_3d =  vstack((points_2d[:,0], zeros_like(points_2d[:, 0]), points_2d[:,1])).T
    rectangle = gl.GLScatterPlotItem(pos=points_3d, color=(0.5, 0.5, 0.5, 0.3), size=0.1)
    rectangle.rotate(*y_up_rotation)
    w.addItem(rectangle)

    #show widget (for different backgroundcolor see PyQtshowmarkers.py)
    w.show()

    # Main Draw Loop (as generator)
    def update_generator():
        last_time = time.time()
        rig_data  = ', '.join('{0}:{1}'.format(body.name, body.color_name) for body in rigs)

        while True:
            m.update()

            # Measure FPS
            new_time = time.time()
            fps = round(1. / (new_time - last_time)) if new_time > last_time else 0
            last_time = new_time

            # Plot
            unident_markers.setData(pos=array(m.get_unident_markers()))
            for rig, scat in zip(rigs, w.items[1:-1]):
                scat.setData(pos=array(rig.point_cloud_markers))

            # Update Title
            w.setWindowTitle('MotivePy Viewer. Rigid Bodies: {rigid_bodies}. Update Rate: {fps} fps'.format(fps=fps,
                                                                                                            rigid_bodies = rig_data))
            # Return Nothing
            yield

    updater = update_generator()

    t = QtCore.QTimer()
    t.timeout.connect(updater.next)
    t.start(2)

    ## Start Qt event loop unless running in interactive mode.
    if __name__ == '__main__':
        import sys
        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            QtGui.QApplication.instance().exec_()