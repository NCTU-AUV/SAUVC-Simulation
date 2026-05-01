from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='orca_auv',
            description='ROS and Gazebo topic namespace for the simulated AUV.',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=namespace,
            parameters=[
                {
                    'config_file': PathJoinSubstitution([
                        FindPackageShare('bringup'),
                        'config',
                        'orca_ros_gz_bridge_config.yaml',
                    ]),
                    'expand_gz_topic_names': True,
                },
            ],
        ),
        Node(
            package='bridge',
            executable='altimeter_to_pressure_sensor_node',
            namespace=namespace,
        ),
    ])
