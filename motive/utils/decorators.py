"""Motive Decorators Module

This module features functionality mainly
in the form of decorators that catch
general exceptions which can arise when one uses
the Motive API.

"""

import time
import motive




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
    def wrapper(*args, **kwargs):
        check=func(*args, **kwargs)
        if check<0:
            raise Exception("Value Not Available. Usually Camera Index Not Valid Or Devices Not Initialized")
        else:
            return check
    return wrapper


def _save_backup(func):
    """Decorator that saves a backup project file"""
    def wrapper(*args, **kwargs):
        func(*args, **kwargs)
        motive.native._save_project(motive.utils.backup_project_filename)
    return wrapper