import os

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

def generate_launch_description():

    urdf_file_name = 'aru_sim.urdf'

    # PATH DEL URDF FILE (LITERALMENTE EL PATH)
    urdf = os.path.join(
        get_package_share_directory('aru_detection'),
        'urdf',
        urdf_file_name)
    
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    robot_state_pub_node = Node(
                            package='robot_state_publisher',
                            executable='robot_state_publisher',
                            name='robot_state_publisher',
                            output='screen',
                            parameters=[{'robot_description': robot_desc}],
                            arguments=[urdf]
                            )
    
    obj_pos_node = Node(
        package='aru_detection',
        executable='ARU_node.py',
        name='aru_node'
    )

    return LaunchDescription([obj_pos_node, robot_state_pub_node])

