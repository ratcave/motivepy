__author__ = 'ratcave'


def check_npresult(func):
    """
    Decorator that checks if the output of a function matches the Motive Error Values, and raises a Python error if so.
    Should decorate every function returning a NPResult type.
    """
    error_dict = {1: (IOError, "File Not Found"),
                  2: (Exception, "Load Failed"),
                  3: (Exception, "Failed"),
                  8: (IOError, "Invalid File"),
                  9: (IOError, "Invalid Calibration File"),
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
    """Generator the creates an iterator that counts back the seconds from total_time"""
    import time
    end_time = time.time() + total_time
    while time.time() < end_time:
        yield end_time - time.time()
    raise StopIteration

def block_for_frame(secs_to_timeout=3):
    """Decorator to continually call a function until it stops raising a RuntimeWarning or until timeout."""
    def decorator_fun(func):
        def wrapper(*args, **kwargs):
            for time in countdown_timer(secs_to_timeout):
                try:
                    return func(*args, **kwargs)
                except RuntimeWarning:
                    pass
            else:
                raise RuntimeWarning("No Frames Available: Timed Out after {} seconds".format(secs_to_timeout))
        return wrapper
    return decorator_fun


def check_cam_setting(func):
    """  Decorator to check if calling a TT_Camera function returns an exception. """
    def wrapper(*args, **kwargs):
        check=func(*args, **kwargs)
        if check<0:
            raise Exception("Value Not Available. Usually Camera Index Not Valid Or Devices Not Initialized")
        else:
            return check
    return wrapper