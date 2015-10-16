__author__ = 'ratcave'

from os import path

# Utility Functions
def check_file_exists(file_name):
    if not path.exists(file_name):
        raise IOError("File '{}' Does not exist.".format(file_name))
    return True

def check_file_extension(file_name, extension='.ttp'):
    if not path.splitext(file_name)[1] == extension:
        raise IOError("File '{}' must have '{}' extension".format(file_name, extension))
