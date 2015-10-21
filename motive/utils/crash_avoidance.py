__author__ = 'ratcave'

import os
import appdirs
from . import decorators

# Utility Functions
def check_file_exists(file_name):
    if not os.path.exists(file_name):
        raise IOError("File '{}' Does not exist.".format(file_name))
    return True

def check_file_extension(file_name, extension='.ttp'):
    if not os.path.splitext(file_name)[1] == extension:
        raise IOError("File '{}' must have '{}' extension".format(file_name, extension))

def get_backup_dir():
    """Checks that the data directory exists, and creates it if not.  Data directory location is platform-specific."""

    data_dir = appdirs.user_data_dir('MotivePy')
    if not os.path.exists(data_dir):
        print("Data Directory not found--creating new data directory at {0}".format(data_dir))
        os.makedirs(data_dir)

    return str(data_dir)

backup_project_filename = os.path.join(get_backup_dir(), 'last_project.ttp')