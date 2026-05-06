import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = get_package_share_directory('bringup')
    world_file = os.path.join(bringup_share, 'worlds', 'water_world.sdf')
    gui_config_file = os.path.join(bringup_share, 'config', 'water_world.config')

    namespace = LaunchConfiguration('namespace')
    arena = LaunchConfiguration('arena')
    seed = LaunchConfiguration('seed')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='orca_auv',
            description='ROS and Gazebo topic namespace for the simulated AUV.',
        ),
        DeclareLaunchArgument(
            'arena',
            default_value='qualification',
            description='Arena profile to spawn: finals or qualification',
        ),
        DeclareLaunchArgument(
            'seed',
            default_value='',
            description='Optional random seed for deterministic prop placement',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py',
                ])
            ]),
            launch_arguments={
                'gz_args': f'{world_file} -r --gui-config {gui_config_file}'
            }.items(),
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
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/world/water_world/create@ros_gz_interfaces/srv/SpawnEntity',
            ],
            output='screen',
        ),
        Node(
            package='bringup',
            executable='entity_spawner.py',
            parameters=[
                {'arena': arena, 'seed': seed},
            ],
            output='screen',
        ),
        Node(
            package='bridge',
            executable='altimeter_to_pressure_sensor_node',
            namespace=namespace,
        ),
    ])
