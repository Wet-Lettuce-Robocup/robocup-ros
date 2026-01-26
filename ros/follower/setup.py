from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'follower'

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
    description='Package for Wet Lettuce line follow robot.',
    license='GPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twist_subscriber = follower.twist_subscriber:main',
            'odom_publisher = follower.odom_publisher:main',
        ],
    },
)
