from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[
                {'config_file': PathJoinSubstitution([FindPackageShare('sauvc_pkg'), 'config', 'sauvc_ros_gz_bridge_config.yaml'])},
            ],
        ),
    ])
