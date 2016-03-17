"""Motive Decorators Module

This module features functionality mainly
in the form of decorators that catch
general exceptions which can arise when one uses
the Motive API.

Example:
    A possible way to implement TT_SaveProject() from the Motive API
    such as to catch its various exceptions encoded by its return type::

    >>>@utils.decorators.check_npresult
    >>>def save_project(str project_file):
    >>>     return TT_SaveProject(project_file)

"""

import time
import motive
import functools

def check_npresult(func):
    """Decorator that checks if the output of a function matches the Motive Error Values, and raises a Python error if so

    Note:
        Should decorate every Motive API function returning a NPResult type.
    """
    error_dict = {1:  (IOError, "File Not Found"),
                  2:  (Exception, "Load Failed"),
                  3:  (Exception, "Failed"),
                  8:  (IOError, "Invalid File"),
                  9:  (IOError, "Invalid Calibration File"),
                  10: (EnvironmentError, "Unable To Initialize"),
                  11: (EnvironmentError, "Invalid License"),
                  14: (RuntimeWarning, "No Frames Available")}
    def wrapper(*args, **kwargs):
        npresult = func(*args, **kwargs)
        if npresult in error_dict:
            error, msg = error_dict[npresult]
            raise error(msg)
    return wrapper


def countdown_timer(total_time):
    """Generator that creates an iterator that counts from total_time to zero

    Args:
        total_time(int): Countdown time in seconds
    """
    end_time = time.time() + total_time
    while time.time() < end_time:
        yield end_time - time.time()
    raise StopIteration


def block_for_frame(secs_to_timeout=3):
    """Decorator to repeatedly call a function until it stops raising a RuntimeWarning or until timeout

    Args:
        secs_to_timeout(int): Seconds the function is repeatedly called if it returns a RuntimeWarning
    """
    def decorator_fun(func):
        @functools.wraps
        def wrapper(*args, **kwargs):
            for t in countdown_timer(secs_to_timeout):
                try:
                    return func(*args, **kwargs)
                except RuntimeWarning:
                    pass
            else:
                raise RuntimeWarning("No Frames Available: Timed Out after {} seconds".format(secs_to_timeout))
        return wrapper
    return decorator_fun


def check_cam_setting(func):
    """Decorator that checks if a TT_Camera function can be called correctly

    Returns:
        check(int): An integer value encoding the camera setting (see camera.pyx)
    Raises:
        Exception: If the camera function returns a value encoding an error
    """
    # @functools.wraps
    def wrapper(*args, **kwargs):
        check=func(*args, **kwargs)
        if check<0:
            raise Exception("Value Not Available. Usually Camera Index Not Valid Or Devices Not Initialized")
        else:
            return check
    return wrapper


def _save_backup(func):
    """Decorator that saves a backup project file"""
    @functools.wraps
    def wrapper(*args, **kwargs):
        func(*args, **kwargs)
        motive.native._save_project(motive.utils.backup_project_filename)
    return wrapper