import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Path to the launch files of different packages
    package_name1 = 'realsense2_camera'
    package_name2 = 'octomap_server2'
    package_name3 = 'my_moveit'

    # Get package directories
    package_dir1 = get_package_share_directory(package_name1)
    package_dir2 = get_package_share_directory(package_name2)
    package_dir3 = get_package_share_directory(package_name3)

    # Specify the path to the launch files
    launch_file1 = os.path.join(package_dir1, 'launch', 'rs_launch.py')
    launch_file2 = os.path.join(package_dir2, 'launch', 'octomap_server_launch.py')
    launch_file3 = os.path.join(package_dir3, 'launch', 'xarm6_planner_realmove.launch.py')

    # Include launch files
    launch_description = LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file1),
            launch_arguments={'depth_module.depth_profile:=1280x720x30'}.items(),
            launch_arguments={'pointcloud.enable:=true'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file2),
            # launch_arguments={'arg2': 'value2'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file3),
            # launch_arguments={'arg2': 'value2'}.items(),
        ),
    ])

    return launch_description

# ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 pointcloud.enable:=true
# ros2 launch octomap_server2 octomap_server_launch.py