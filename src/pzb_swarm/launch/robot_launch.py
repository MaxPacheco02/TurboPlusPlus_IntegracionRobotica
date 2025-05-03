import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Uncomment if this is the highest launch file
    ns_arg = DeclareLaunchArgument('ns_', default_value='pzb')

    visual = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('pzb_description'),
            '/launch/rviz_launch.py'
        ]),
        launch_arguments={'ns_': LaunchConfiguration('ns_')}.items(),
    )

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('pzb_control'),
            '/launch/pzb_control_launch.py'
        ]),
        launch_arguments={'ns_': LaunchConfiguration('ns_')}.items(),
    )

    return LaunchDescription([
        # Uncomment if this is the highest launch file
        ns_arg,
        visual,
        control,
    ])