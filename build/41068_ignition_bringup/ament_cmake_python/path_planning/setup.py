from setuptools import find_packages
from setuptools import setup

setup(
    name='path_planning',
    version='1.0.3',
    packages=find_packages(
        include=('path_planning', 'path_planning.*')),
)
