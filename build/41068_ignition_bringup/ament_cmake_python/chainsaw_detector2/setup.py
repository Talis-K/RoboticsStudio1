from setuptools import find_packages
from setuptools import setup

setup(
    name='chainsaw_detector2',
    version='1.0.3',
    packages=find_packages(
        include=('chainsaw_detector2', 'chainsaw_detector2.*')),
)
