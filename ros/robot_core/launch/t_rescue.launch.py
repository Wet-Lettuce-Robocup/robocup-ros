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
