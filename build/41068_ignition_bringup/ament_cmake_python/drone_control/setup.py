from setuptools import find_packages
from setuptools import setup

setup(
    name='drone_control',
    version='1.0.3',
    packages=find_packages(
        include=('drone_control', 'drone_control.*')),
)
