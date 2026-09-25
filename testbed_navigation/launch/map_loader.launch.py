import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('testbed_navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    default_map = os.path.join(
        get_package_share_directory('testbed_bringup'), 'maps', 'testbed_world.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('map_params_file',
                              default_value=os.path.join(pkg, 'config', 'map_server_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[LaunchConfiguration('map_params_file'),
                        {'use_sim_time': use_sim_time,
                         'yaml_filename': LaunchConfiguration('map')}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'autostart': LaunchConfiguration('autostart'),
                         'node_names': ['map_server']}],
        ),
    ])
