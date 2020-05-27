# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    version='1.0.0',
    packages=['aruco_analyzer'],
    package_dir={'': 'src'},
    scripts=[
        'scripts/create_marker.py',
        'scripts/create_grid.py',
        'scripts/create_cube.py',
    ],
    install_requires=[
        'opencv-contrib-python',
        'numpy',
        'pyquaternion',
    ]
)

setup(**setup_args)
