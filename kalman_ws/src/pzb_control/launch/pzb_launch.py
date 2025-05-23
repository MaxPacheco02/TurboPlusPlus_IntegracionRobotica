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
    
    rviz_config = os.path.join(get_package_share_directory("pzb_control"),"rviz/","rviz.rviz")
    pzb_config = os.path.join(
        get_package_share_directory('pzb_control'),
        'config',
        'conf.yaml'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
    )

    state_node = Node(
        package='pzb_control',
        executable='state_estimate',
        parameters=[pzb_config],

        # Comment remappings for closed loop (REAL LIFE)
        # Include remappings for open loop (SIMULATIONS AND TESTING)
        remappings=[
            ('/VelocityEncL', '/VelocitySetL'),
            ('/VelocityEncR', '/VelocitySetR'),
        ],
    )

    guidance_node = Node(
        package='pzb_control',
        executable='guidance_node',
        parameters=[pzb_config],
    )


    line_follower_path_publisher_node = Node(
        package='pzb_control',
        executable='line_follower_path_publisher_node',
    )

    pzb_teleop_node = Node(
        package='pzb_control',
        executable='pzb_teleop_node.py',
    )

    path_publisher_node = Node(
        package='pzb_control',
        executable='path_publisher_node.py',
    )

    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('pzb_vision'),
                'launch',
                'pzb_vision_launch.py'
            ])
        ]),
    )

    foxglove_bridge = Node(
        name="foxglove_bridge",
        package="foxglove_bridge",
        executable="foxglove_bridge")

    return LaunchDescription([
        state_node,
        guidance_node,
        # rviz_node,
        line_follower_path_publisher_node,
        vision_launch,
        # pzb_teleop_node, # NO AQUI, POR VENTANA APARTE JE

        # path_publisher_node,
        foxglove_bridge,
    ])
