from distutils.core import setup

setup(
    version='0.0.0',
    scripts=['scripts/talker.py', 'scripts/hit_spider_planner.py'],
    packages=['fast_legged_planner_py'],
    package_dir={'': './'}
)
