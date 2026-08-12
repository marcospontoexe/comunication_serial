from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'comunication_serial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marcos Daniel Santana',
    maintainer_email='marcos.daniel1990@hotmail.com',
    description='Reliable serial bridge between ROS 2 and an ESP32',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'comunication_serial_executable = comunication_serial.communicator_serial:main'
        ],
    },
)
