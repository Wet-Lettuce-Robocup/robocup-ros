import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 1. Setup paths (Change 'my_robot_simulation' to your actual package name if applicable)
    # If not using a package, you can hardcode absolute paths using os.path.join()
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_dir = get_package_share_directory('robot_core')

    # Define where your model and media folders live so Gazebo can find them
    # Replace these with your actual local paths
    world_file = os.path.join(pkg_dir, 'worlds', 'my_world.sdf')
    models_dir = os.path.join(pkg_dir, 'models')
    media_dir = os.path.join(pkg_dir, 'media')

    # 2. Set Environment Variables so Gazebo knows where to look for textures/models
    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', value=[models_dir, ':', media_dir]
    )

    # 3. Include the Gazebo Sim launch description (starts the simulator)
    # We load an empty world by default ("empty.sdf")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items(),
    )

    # 4. Spawn your 4-wheeled robot into Gazebo from your SDF file
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',
            'four_wheeled_robot',
            '-file',
            os.path.join(models_dir, 'robot.sdf'),
            '-z',
            '0.1',  # Spawn slightly off the ground so it doesn't clip
        ],
        output='screen',
    )

    # 5. Start the ROS <-> Gazebo Parameter Bridge for cmd_vel, odom, and cameras
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/image_raw/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/down_camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/down_camera/image_raw/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        output='screen',
    )

    return LaunchDescription([set_resource_path, gz_sim, spawn_robot, ros_gz_bridge])
