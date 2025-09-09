from setuptools import find_packages
from setuptools import setup

setup(
    name='subpackage_1',
    version='1.0.3',
    packages=find_packages(
        include=('subpackage_1', 'subpackage_1.*')),
)
