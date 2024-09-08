from distutils.core import setup

setup(
    version='0.0.0',
    scripts=['scripts/hit_spider_planner.py',
             'scripts/elspider_air_swingtraj_planner.py',
             'scripts/traj_opt_demo.py',
             'scripts/gridmap_sub.py',
             'scripts/pointcloud_parser.py',
             'scripts/test_elspider_air.py'],
    packages=['fast_legged_planner_py'],
    package_dir={'': './'}
)
