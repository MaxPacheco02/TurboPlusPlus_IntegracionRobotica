import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    urdf_file_name = 'pz_sim.urdf'

    # PATH DEL URDF FILE (LITERALMENTE EL PATH)
    urdf = os.path.join(
        get_package_share_directory('puzzlebot_sim2'),
        'urdf',
        urdf_file_name)
    
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    robot_state_pub_node = Node(
                            package='robot_state_publisher',
                            executable='robot_state_publisher',
                            name='robot_state_publisher',
                            output='screen',
                            parameters=[{'robot_description': robot_desc}, {'use_tf_static': True}],
                            arguments=[urdf]
                            )
    
    joint_state_publisher_node = Node(
                            package='puzzlebot_sim2',
                            executable='joint_pub',
                            output='screen'
                            )

    l_d = LaunchDescription([robot_state_pub_node, joint_state_publisher_node])

    return l_d