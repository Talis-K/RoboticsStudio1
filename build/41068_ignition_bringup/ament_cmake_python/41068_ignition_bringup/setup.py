from setuptools import find_packages
from setuptools import setup

setup(
    name='41068_ignition_bringup',
    version='1.0.3',
    packages=find_packages(
        include=('41068_ignition_bringup', '41068_ignition_bringup.*')),
)
