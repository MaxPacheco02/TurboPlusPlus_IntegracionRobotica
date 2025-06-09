import os

from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    kalman_node = Node(
        package='pzb_kalman',
        executable='dead_reckoning',
    )

    return LaunchDescription([kalman_node])