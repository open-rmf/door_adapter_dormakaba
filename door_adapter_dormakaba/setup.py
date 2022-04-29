from setuptools import setup
import os
from glob import glob

package_name = 'door_adapter_dormakaba'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name,['config.yaml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name,'launch'), glob('launch/*.launch.xml')),
    ],
    install_requires=['setuptools'],
    author='alex',
    author_email='alex_chua@artc.a-star.edu.sg',
    zip_safe=True,
    description='RMF Door Adapter for Dormakaba doors',
    license='Apache License Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'door_adapter_dormakaba = door_adapter_dormakaba.door_adapter:main',
        ],
    },
)
