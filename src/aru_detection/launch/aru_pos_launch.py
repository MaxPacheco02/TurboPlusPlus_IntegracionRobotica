import os

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

def generate_launch_description():

    obj_pos_node = Node(
        package='aru_detection',
        executable='ARU_node.py',
        name='aru_node'
    )

    aru_way_node = Node(
        package='aru_detection',
        executable='ARU_WAY_node.py',
        name='way_node'
    )

    kalman_local_node = Node(
        package='aru_detection',
        executable='aru_exe',
        name='aru_node',
        output='screen' 
    )


    # obj_pos_node, aru_way_node, 
    return LaunchDescription([kalman_local_node])

