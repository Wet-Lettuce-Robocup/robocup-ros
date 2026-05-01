import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robot_core'), 'config', 'params.yaml'
    )

    return LaunchDescription(
        [
            Node(
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
            ),
            Node(
                package='robot_core',
                executable='i2c_controller',
                name='i2c_controller',
                output='screen',
                parameters=[config],
            ),
            Node(
                package='bno08x_driver',
                executable='bno08x_driver',
                name='bno08x_driver',
                output='screen',
                parameters=[config],
            ),
            # Servos
            Node(
                package='robot_core',
                executable='servo_controller',
                name='servo_grab',
                output='screen',
                parameters=[config],
            ),
            Node(
                package='robot_core',
                executable='servo_controller',
                name='servo_lift',
                output='screen',
                parameters=[config],
            ),
            Node(
                package='robot_core',
                executable='servo_controller',
                name='servo_tray_release',
                output='screen',
                parameters=[config],
            ),
            Node(
                package='robot_core',
                executable='odom_publisher',
                name='odom_publisher',
                output='screen',
                parameters=[config],
            ),
            Node(
                package='robot_core',
                executable='twist_subscriber',
                name='twist_subscriber',
                output='screen',
                parameters=[config],
            ),
            Node(
                package='robot_core',
                executable='status_led',
                name='status_led',
                output='screen',
                parameters=[config],
            ),
            Node(
                package='robot_core',
                executable='oled_controller',
                name='oled_controller',
                output='screen',
                parameters=[config],
            ),
            Node(
                package='camera_ros',
                executable='camera_node',
                name='camera',
                output='screen',
                parameters=[config],
            ),
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[config],
            ),
            LifecycleNode(
                package='robot_core',
                executable='state_machine',
                name='state_machine',
                namespace='',
                output='screen',
                parameters=[config],
            ),
        ]
    )
