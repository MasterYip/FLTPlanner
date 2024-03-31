import os
import glob
import subprocess

# Directory Management
try:
    # Run in Terminal
    ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
except:
    # Run in ipykernel & interactive
    ROOT_DIR = os.getcwd()


def prof2png(file_dir):
    png_file = os.path.splitext(file_dir)[0] + '.png'
    command = f'gprof2dot -f pstats {file_dir} | dot -Tpng -o {png_file}'
    subprocess.run(command, shell=True)


if __name__ == "__main__":
    # Convert .prof files to PNG
    prof_files = glob.glob(os.path.join(ROOT_DIR, 'log', '*.prof'))
    for prof_file in prof_files:
        prof2png(prof_file)
