from setuptools import find_packages
from setuptools import setup

setup(
    name='order_interface',
    version='0.0.0',
    packages=find_packages(
        include=('order_interface', 'order_interface.*')),
)
