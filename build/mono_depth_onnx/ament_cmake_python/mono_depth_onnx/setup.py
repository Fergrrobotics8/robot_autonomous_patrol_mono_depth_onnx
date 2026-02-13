from setuptools import find_packages
from setuptools import setup

setup(
    name='mono_depth_onnx',
    version='1.0.0',
    packages=find_packages(
        include=('mono_depth_onnx', 'mono_depth_onnx.*')),
)
