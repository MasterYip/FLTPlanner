import cProfile
import pstats
import os
from .prof2png import prof2png

# Directory Management
try:
    # Run in Terminal
    ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
except:
    # Run in ipykernel & interactive
    ROOT_DIR = os.getcwd()

# Default Settings
DO_PROF = True
VERBOSE = True
SAVE = False

if not os.path.isdir(os.path.join(ROOT_DIR, "log")):
    os.mkdir(os.path.join(ROOT_DIR, "log"))


def do_cprofile(filename=None, do_prof=DO_PROF, verbose=VERBOSE, save=SAVE):
    """
    Decorator for function profiling.
    """
    def wrapper(func):
        def profiled_func(*args, **kwargs):
            # Flag for do profiling or not.
            if do_prof:
                profile = cProfile.Profile()
                profile.enable()
                result = func(*args, **kwargs)
                profile.disable()
                # Sort stat by internal time.
                sortby = "tottime"
                ps = pstats.Stats(profile).strip_dirs().sort_stats(sortby)
                if save:
                    if filename:
                        ps.dump_stats(filename)
                        prof2png(filename)
                    else:
                        default_name = os.path.join(
                            ROOT_DIR, "log", func.__name__+".prof")
                        ps.dump_stats(default_name)
                        prof2png(default_name)
                if verbose:
                    ps.print_stats(10, 1.0, '.*')
            else:
                result = func(*args, **kwargs)
            return result
        return profiled_func
    return wrapper
