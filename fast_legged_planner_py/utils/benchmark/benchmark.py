import cProfile
import pstats
import os

# Settings
DO_PROF = True
PRINT = True


def do_cprofile(filename=None):
    """
    Decorator for function profiling.
    """
    def wrapper(func):
        def profiled_func(*args, **kwargs):
            # Flag for do profiling or not.
            # DO_PROF = os.getenv("PROFILING")
            if DO_PROF:
                profile = cProfile.Profile()
                profile.enable()
                result = func(*args, **kwargs)
                profile.disable()
                # Sort stat by internal time.
                sortby = "tottime"
                ps = pstats.Stats(profile).strip_dirs().sort_stats(sortby)
                if filename:
                    ps.dump_stats(filename)
                if PRINT:
                    ps.print_stats(10, 1.0, '.*')
            else:
                result = func(*args, **kwargs)
            return result
        return profiled_func
    return wrapper
