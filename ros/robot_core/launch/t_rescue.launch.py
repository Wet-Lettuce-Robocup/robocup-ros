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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    robot_core_pkg = get_package_share_directory('robot_core')
    robot_core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_core_pkg, 'launch', 'state_manager.launch.py')
        )
    )

    t_rescue_pkg = get_package_share_directory('ml_rescue')
    t_rescue_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(t_rescue_pkg, 'launch', 'ml_rescue.launch.py')
        )
    )

    return LaunchDescription([robot_core_launch, t_rescue_launch])
