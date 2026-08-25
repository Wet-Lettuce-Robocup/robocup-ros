# Robot Core
# Copyright (C) 2026  Dry Lettuce
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'robot_core'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            os.path.join('share', 'ament_index', 'resource_index', 'packages'),
            [os.path.join('resource', package_name)],
        ),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer="William D'Olier",
    maintainer_email='william@dolier.net',
    description='Core package for Wet Lettuce line follow robot.',
    license='GPL',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'twist_subscriber = robot_core.twist_subscriber:main',
            'odom_publisher = robot_core.odom_publisher:main',
            'servo_controller = robot_core.servo_controller:main',
            'state_machine = robot_core.state_machine:main',
            'status_led = robot_core.status_led:main',
            'i2c_controller = robot_core.i2c_controller:main',
            'oled_controller = robot_core.oled_controller:main',
            'button_publisher = robot_core.button_publisher:main',
            'pwm_controller = robot_core.pwm_controller:main',
            'fan_controller = robot_core.fan_controller:main',
            'front_led_controller = robot_core.front_led_controller:main',
            'movement_actions = robot_core.movement_actions:main',
        ],
    },
)
