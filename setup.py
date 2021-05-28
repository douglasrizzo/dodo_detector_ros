## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['dodo_detector_ros'],
    package_dir={'': 'src'},
    dependency_links = ['https://github.com/douglasrizzo/dodo_detector@0.8']
)

setup(**setup_args)
