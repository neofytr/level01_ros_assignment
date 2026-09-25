import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SERVERS = [
    ('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
    ('nav2_velocity_smoother', 'velocity_smoother',
     [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
    ('nav2_planner', 'planner_server', []),
    ('nav2_behaviors', 'behavior_server', []),
    ('nav2_bt_navigator', 'bt_navigator', []),
]


def generate_launch_description():
    pkg = get_package_share_directory('testbed_navigation')
    params = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    servers = [
        Node(package=package, executable=executable, name=executable, output='screen',
             parameters=[params, {'use_sim_time': use_sim_time}], remappings=remappings)
        for package, executable, remappings in SERVERS
    ]

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': LaunchConfiguration('autostart'),
            'node_names': [executable for _, executable, _ in SERVERS],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(pkg, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        *servers,
        lifecycle_manager,
    ])
