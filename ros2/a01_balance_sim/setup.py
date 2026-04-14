import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'a01_balance_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='umas',
    maintainer_email='1970313791@qq.com',
    description='Gazebo Sim integration package for the a01 balance ball-on-beam robot.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [],
    },
)
