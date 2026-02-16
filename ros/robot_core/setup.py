from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'robot_core'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer="William D'Olier",
    maintainer_email='william@dolier.net',
    description='Core package for Wet Lettuce line follow robot.',
    license='GPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twist_subscriber = robot_core.twist_subscriber:main',
            'odom_publisher = robot_core.odom_publisher:main',
            'state_machine = robot_core.state_machine:main',
        ],
    },
)
