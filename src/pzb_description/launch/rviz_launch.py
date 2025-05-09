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
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
    
def generate_launch_description():
    
    rviz_config = os.path.join(get_package_share_directory("pzb_description"),"rviz/","rviz2.rviz")

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
    )

    tf2 = Node(
        package="pzb_description",
        executable="pzb_tf2_publisher",
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    path_urdf = os.path.join(get_package_share_directory("pzb_description"), "urdf/", "pzb.urdf.xacro")
    robot_description_content = ParameterValue(Command(['xacro ', path_urdf]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{ 
            'robot_description': robot_description_content}]
    )

    return LaunchDescription([
        rviz_node,
        tf2,
        robot_state_publisher_node,
        # joint_state_publisher_node,
    ])
