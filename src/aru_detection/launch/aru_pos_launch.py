import os

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

def generate_launch_description():

    aru_config = os.path.join(
        get_package_share_directory('aru_detection'),
        'config/',
        'aruco_map.yaml',
    )

    aru_tf2_publisher = Node(
        package='aru_detection',
        executable='aru_tf2_exe',
        name='aruco_tf_publisher',
        parameters=[aru_config],
        output='screen',
    )

    obj_pos_node = Node(
        package='aru_detection',
        executable='aru_exe',
        name='aru_node',
        output='screen', 
    )

    aru_for_pallets = Node(
        package='aru_detection',
        executable='aru_for_pallets',
        output='screen',
    )

    # obj_pos_node, aru_way_node, 
    return LaunchDescription([obj_pos_node, aru_tf2_publisher, aru_for_pallets])

