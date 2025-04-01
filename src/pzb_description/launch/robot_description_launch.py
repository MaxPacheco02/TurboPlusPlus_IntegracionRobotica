import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():


    path_urdf = os.path.join(get_package_share_directory("pzb_description"), "urdf/", "pzb.urdf.xacro")

    rviz_config = os.path.join(get_package_share_directory("pzb_description"),'rviz/','rviz.rviz')

    robot_description_content = ParameterValue(Command(['xacro ', path_urdf]), value_type=str)

    test_urdf_node = Node(
        package='pzb_description',
        executable='test_urdf',
        output='screen'
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{ 
            'robot_description': robot_description_content}]
    )

    test_joy_teleop = Node(
            package='pzb_description',
            executable='test_joy_teleop.py'
    )


    joy_params_file = os.path.join(
        get_package_share_directory('pzb_description'),
        'config', 'joy_params.yaml'
    )
    
    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_params_file],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        test_urdf_node,
        test_joy_teleop,
        joy_node,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])
