from distutils.core import setup

setup(
    version='0.0.0',
    scripts=['scripts/hit_spider_planner.py',
             'scripts/gridmap_sub.py',
             'scripts/image_publisher.py',
             'scripts/pointcloud_parser.py',],
    packages=['fast_legged_planner_py'],
    package_dir={'': './'}
)
