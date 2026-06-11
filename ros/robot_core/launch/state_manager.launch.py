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
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robot_core'), 'config', 'params.yaml'
    )

    oled_controller = Node(
        package='robot_core',
        executable='oled_controller',
        name='oled_controller',
        output='screen',
        parameters=[config],
    )

    static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_static_tf',
        arguments=[
            '--x',
            '0',
            '--y',
            '0',
            '--z',
            '0',  # x y z
            '--roll',
            '0',
            '--pitch',
            '0',
            '--yaw',
            '0',  # roll pitch yaw
            '--frame-id',
            'base_link',
            '--child-frame-id',
            'imu_link',
        ],
    )

    i2c_controller = Node(
        package='robot_core',
        executable='i2c_controller',
        name='i2c_controller',
        output='screen',
        parameters=[config],
    )

    bno08x_driver = Node(
        package='bno08x_driver',
        executable='bno08x_driver',
        name='bno08x_driver',
        output='screen',
        parameters=[config],
    )

    # Servos
    servo_grab = Node(
        package='robot_core',
        executable='servo_controller',
        name='servo_grab',
        output='screen',
        parameters=[config],
    )

    servo_lift = Node(
        package='robot_core',
        executable='servo_controller',
        name='servo_lift',
        output='screen',
        parameters=[config],
    )

    servo_tray_release = Node(
        package='robot_core',
        executable='servo_controller',
        name='servo_tray_release',
        output='screen',
        parameters=[config],
    )

    odom_publisher = Node(
        package='robot_core',
        executable='odom_publisher',
        name='odom_publisher',
        output='screen',
        parameters=[config],
    )
    twist_subscriber = Node(
        package='robot_core',
        executable='twist_subscriber',
        name='twist_subscriber',
        output='screen',
        parameters=[config],
    )
    pwm_0_controller = Node(
        package='robot_core',
        executable='pwm_controller',
        name='pwm_controller',
        namespace='pwm0',
        output='screen',
        parameters=[config],
    )
    pwm_1_controller = Node(
        package='robot_core',
        executable='pwm_controller',
        name='pwm_controller',
        namespace='pwm1',
        output='screen',
        parameters=[config],
    )
    pwm_2_controller = Node(
        package='robot_core',
        executable='pwm_controller',
        name='pwm_controller',
        namespace='pwm2',
        output='screen',
        parameters=[config],
    )
    status_led = Node(
        package='robot_core',
        executable='status_led',
        name='status_led',
        output='screen',
        parameters=[config],
    )
    idle_button = Node(
        package='robot_core',
        executable='button_publisher',
        name='idle_button',
        output='screen',
        parameters=[config],
    )
    fan_controller = Node(
        package='robot_core',
        executable='fan_controller',
        name='fan_controller',
        output='screen',
        parameters=[config],
    )
    front_camera = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera_node',
        namespace='front_camera',
        output='screen',
        parameters=[config],
    )
    down_camera = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera_node',
        namespace='down_camera',
        output='screen',
        parameters=[config],
    )
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config],
    )
    movement_action_node = Node(
        package='robot_core',
        executable='movement_actions',
        name='movement_actions',
        output='screen',
        parameters=[config],
    )

    line_follow_pkg = get_package_share_directory('line_follow')
    line_follow_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(line_follow_pkg, 'launch', 'line_follow.launch.py')
        )
    )
    state_machine = Node(
        package='robot_core',
        executable='state_machine',
        name='state_machine',
        namespace='',
        output='screen',
        parameters=[config],
    )
    return LaunchDescription(
        [
            oled_controller,
            static_transform,
            i2c_controller,
            bno08x_driver,
            servo_grab,
            servo_lift,
            servo_tray_release,
            odom_publisher,
            twist_subscriber,
            movement_action_node,
            pwm_0_controller,
            pwm_1_controller,
            pwm_2_controller,
            status_led,
            idle_button,
            fan_controller,
            front_camera,
            down_camera,
            ekf_node,
            line_follow_launch,
            state_machine,
        ]
    )
