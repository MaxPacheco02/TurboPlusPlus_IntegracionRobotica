import os

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    state_machine_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('pzb_master'),
                'launch',
                'state_machine_launch.py'
            ])
        ]),
    )

    aru_pose_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('aru_detection'),
                'launch',
                'aru_pose_launch.py'
            ])
        ]),
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('pzb_description'),
                'launch',
                'rviz_launch.py'
            ])
        ]),
    )

    pid_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('pzb_control'),
                'launch',
                'pid_launch.py'
            ])
        ]),
    )

    pzb_kalman_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('pzb_kalman'),
                'launch',
                'pzb_kalman_launch.py'
            ])
        ]),
    )

    a_star_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('pzb_control'),
                'launch',
                'a_star_launch.py'
            ])
        ]),
    )

    laser_preprocess = Node(
        package='pzb_localization',
        executable='laser_preprocess',
    )

    alignment_node = Node(
        package='pzb_vision',
        executable='alignment_node',
    )

    qr_detector = Node(
        package='qr_detector_pkg',
        executable='qr_detector',
    )

    trailer_yolo_node = Node(
        package='pzb_vision',
        executable='trailer_yolo_node.py',
    )
    
    return LaunchDescription([
        trailer_yolo_node,
        # qr_detector,
        # alignment_node,
        laser_preprocess,
        
        a_star_launch,
        pzb_kalman_launch,
        pid_launch,
        rviz_launch,
        # aru_pose_launch,
        # state_machine_launch,
    ])
