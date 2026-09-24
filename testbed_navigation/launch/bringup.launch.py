import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('testbed_navigation')
    description = get_package_share_directory('testbed_description')
    gazebo = get_package_share_directory('testbed_gazebo')

    use_sim_time = LaunchConfiguration('use_sim_time')

    def include(share, name, **arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', name)),
            launch_arguments=arguments.items())

    def stage(name, params):
        return include(pkg, f'{name}.launch.py', use_sim_time=use_sim_time,
                       params_file=os.path.join(pkg, 'config', params))

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(pkg, 'rviz', 'navigation.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        include(description, 'robot_description.launch.py'),
        include(gazebo, 'spawn_playground.launch.py'),
        include(gazebo, 'spawn_testbed.launch.py'),
        stage('localization', 'amcl_params.yaml'),
        stage('navigation', 'nav2_params.yaml'),
        rviz,
    ])
