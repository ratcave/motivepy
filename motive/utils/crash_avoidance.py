"""Motive Crash Avoidance Module

This module features functionality to
avoid crashes related to not existing
files or wrong file extensions.
"""
from __future__ import absolute_import

import os
import appdirs
import warnings
from . import decorators

# Utility Functions
def check_file_exists(file_name):
    """Returns true if file exists

    Args:
        file_name(str): The name of the file
    Raises:
        IOError: If the file does not exist
    """
    if not os.path.exists(file_name):
        raise IOError("File '{}' Does not exist.".format(file_name))
    return True

def check_file_extension(file_name, extension='.ttp'):
    """Raises IOError if file does not have correct extension

    Args:
        file_name(str): Name of the file
        extension(Optional[str]): Extension of the file
    """
    if not os.path.splitext(file_name)[1] == extension:
        raise IOError("File '{}' must have '{}' extension".format(file_name, extension))

def get_backup_dir():
    """Returns name of data directory

    Checks that the data directory exists and creates it if not.

    Note:
        Data directory location is platform-specific.
    """

    data_dir = appdirs.user_data_dir('MotivePy')
    if not os.path.exists(data_dir):
        warnings.warn("Data Directory not found--creating new data directory at {0}".format(data_dir))
        os.makedirs(data_dir)

    return str(data_dir)

backup_project_filename = os.path.join(get_backup_dir(), 'last_project.ttp')
