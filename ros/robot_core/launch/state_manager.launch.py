from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_core',
            executable='odom_publisher',
            name='odom_publisher',
            output='screen',
        ),

        Node(
            package='robot_core',
            executable='twist_subscriber',
            name='twist_subscriber',
            output='screen',
        ),

        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            output='screen',
        ),

        LifecycleNode(
            package='robot_core',
            executable='state_machine',
            name='state_machine',
            namespace='',
            output='screen',
        )
    ])
