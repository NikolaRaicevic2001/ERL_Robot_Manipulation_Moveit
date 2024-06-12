#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node  

def generate_launch_description():
    # Path to the launch files of different packages
    package_name1 = 'realsense2_camera'
    package_name2 = 'my_moveit'
    package_name3 = 'my_moveit'

    # Get package directories
    package_dir1 = get_package_share_directory(package_name1)
    package_dir2 = get_package_share_directory(package_name2)
    package_dir3 = get_package_share_directory(package_name3)

    # Specify the path to the launch files
    launch_file1 = os.path.join(package_dir1, 'launch', 'rs_launch.py')
    launch_file2 = os.path.join(package_dir2, 'launch', 'octomap_server_launch.py')
    launch_file3 = os.path.join(package_dir3, 'launch', 'xarm6_planner_fake.launch.py')

   # Create a node for the static_transform_publisher
    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.11102413' ,'-0.06191412'  ,'0.70101589', '0.18104651', '0.96063305', '0.07411852', 'camera_link', 'world']
    )

    # Include launch files
    launch_description = LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file1),
            launch_arguments={
                'depth_module.depth_profile': '1280x720x30',
                'pointcloud.enable': 'true'
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file2),
            # launch_arguments={'arg2': 'value2'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file3),
            # launch_arguments={'arg2': 'value2'}.items(),
        ),
        # Add the static_transform_publisher node to the launch description
        static_transform_publisher_node
    ])

    return launch_description

# ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 pointcloud.enable:=true
# ros2 launch my_moveit octomap_server_launch.py
# ros2 run tf2_ros static_transform_publisher -0.243 -0.225 1.162 -0.9119 -0.0267 2.0038 camera_link world
