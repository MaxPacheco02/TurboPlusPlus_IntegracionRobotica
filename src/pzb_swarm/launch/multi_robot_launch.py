import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('pzb_swarm'),
            '/launch/robot_launch.py'
        ]),
        launch_arguments={'ns_': 'pzb1'}.items(),
    )

    robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('pzb_swarm'),
            '/launch/robot_launch.py'
        ]),
        launch_arguments={'ns_': 'pzb2'}.items(),
    )

    robot3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('pzb_swarm'),
            '/launch/robot_launch.py'
        ]),
        launch_arguments={'ns_': 'pzb3'}.items(),
    )


    return LaunchDescription([
        robot1,
        robot2,
        robot3,
    ])