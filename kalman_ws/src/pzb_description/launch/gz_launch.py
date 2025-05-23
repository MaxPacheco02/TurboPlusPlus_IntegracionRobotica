# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_pzb_description = get_package_share_directory('pzb_description')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': PathJoinSubstitution([
            pkg_pzb_description,
            'worlds',
            'pzb_world.sdf'
        ])}.items(),
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
                   '/pzb/odom@nav_msgs/msg/Odometry@gz.msgs.OdometryWithCovariance',
                   '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                   '/pzb_gz_drive@geometry_msgs/msg/Twist@gz.msgs.Twist',  
                   '/gz_joints@sensor_msgs/msg/JointState@gz.msgs.Model',
                   ],
        remappings=[
            ('/pzb_gz_drive', '/cmd_vel'),
        ],
        # parameters=[{
        #     'qos_overrides./model/my_roboboat.subscriber.reliability': 'reliable',
        #             }],
        output='screen',
    )

    gz_joint_broadcaster = Node(
        package="pzb_description",
        executable="gz_joint_broadcaster",
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        gz_joint_broadcaster,
    ])