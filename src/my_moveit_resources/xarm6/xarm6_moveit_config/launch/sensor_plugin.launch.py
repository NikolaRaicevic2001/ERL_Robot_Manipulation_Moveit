from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = get_package_share_directory('my_moveit_xarm6_moveit_config') + '/config/sensors.yaml'

    return LaunchDescription([
        Node(
            package='my_moveit_xarm6_moveit_config',
            executable='sensor_node',
            name='sensor_node',
            parameters=[config_file]
        )
    ])

