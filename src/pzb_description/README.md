# Package for Puzzlebot Description

## Setup

Used docker (probably not necessary) because had problems with ros-gazebo bridges. Using from [source ros_gz humble](https://github.com/gazebosim/ros_gz/tree/humble).

- Modified `gz_translator.cpp` to take the time from **/gz/joints**. If not wheels and lidar were not displayed according to urdf.
- Modified `pzb_tf2_publisher.cpp` to take the time for **odom**. If not couldn't visualize **/lidar** and **odom** in Rviz in <em>odom</em> frame.


## Useful commands:

```Shell
// Publish Ros
ros2 topic pub /pzb_gz_drive geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

// Publish Gazebo
gz topic -t /pzb_gz_drive -m gz.msgs.Twist -p 'linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}'
```
