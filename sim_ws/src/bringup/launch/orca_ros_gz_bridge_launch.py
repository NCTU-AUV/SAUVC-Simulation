import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def gz_sim_launch(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower()
    gui_config_file = os.path.join(
        get_package_share_directory('bringup'),
        'config',
        'water_world.config',
    )

    gz_args = f'{world} -r'
    if headless in ('true', '1', 'yes', 'on'):
        gz_args = f'{gz_args} -s'
    else:
        gz_args = f'{gz_args} --gui-config {gui_config_file}'

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py',
                ])
            ]),
            launch_arguments={'gz_args': gz_args}.items(),
        ),
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory('bringup')
    world_file = os.path.join(bringup_share, 'worlds', 'water_world.sdf')

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
            'world',
            default_value=world_file,
            description='Gazebo world file to load.',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Launch Gazebo server-only when true; launch GUI when false.',
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
        OpaqueFunction(function=gz_sim_launch),
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
                # seed 必須明確指定成字串。entity_spawner 把它宣告成 STRING
                # （空字串代表「不指定 seed」），但 launch 預設會對
                # seed:=42 這種輸入做型別推斷、送出 INTEGER，節點會以
                # InvalidParameterTypeException 直接死掉。
                {
                    'arena': ParameterValue(arena, value_type=str),
                    'seed': ParameterValue(seed, value_type=str),
                },
            ],
            output='screen',
        ),
        Node(
            package='bridge',
            executable='altimeter_to_pressure_sensor_node',
            namespace=namespace,
        ),
    ])
