from setuptools import find_packages
from setuptools import setup

setup(
    name='lidar_processing',
    version='1.0.3',
    packages=find_packages(
        include=('lidar_processing', 'lidar_processing.*')),
)
