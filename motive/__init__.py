__author__ = 'nico'

from native import *
from camera import *
from rigidbody import *
from pointcloudgroup import *
import utils
from utils.viewer import show_viewer
from utils.c3d_btk_writer import *
import warnings

try:
    native._initialize()
except EnvironmentError, e:
    warnings.warn('\n' + repr(e))
    pass
