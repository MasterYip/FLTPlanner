# Fast-Legged-Planner-Test

Test repo for Fast-Legged-Planner

## Setup

### Robotic Course Setup

The origin repo is [here](https://github.com/ymontmarin/_tps_robotique)

On Linux or OSX only: https://docs.conda.io/en/latest/miniconda.html

Only a little snippet is applied tou your home .bashrc, everything else will be segmented !

Or use Anaconda on Windows and use conda prompt insteal of a terminal.

Go into the repository folder.

Open a terminal.

Create and activate conda environment:

```bash
conda env create -f robotic_course_env.yml
```

From now on, when you want to work on the TPs you only need to do:

```bash
conda activate robotic_course
```

Meshcat visualisation can be access in full page in `localhost:700N/static/` where N denotes the Nth meshcat instance created with the running kernel.
