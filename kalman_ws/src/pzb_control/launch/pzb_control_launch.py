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
    
    pzb_config = os.path.join(
        get_package_share_directory('pzb_control'),
        'config',
        'conf.yaml'
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

    return LaunchDescription([
        state_node,
    ])
