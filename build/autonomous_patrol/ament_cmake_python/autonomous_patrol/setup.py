from setuptools import find_packages
from setuptools import setup

setup(
    name='autonomous_patrol',
    version='0.0.1',
    packages=find_packages(
        include=('autonomous_patrol', 'autonomous_patrol.*')),
)
