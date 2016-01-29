__author__ = 'nico'

from native import *
from camera import *
from rigidbody import *
import utils
from utils.viewer import show_viewer
from utils.c3d_btk_writer import *

try:
    native._initialize()
except EnvironmentError:
    print('Warning: License not found, so no connection made.')
    pass
