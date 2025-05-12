# Package for Puzzlebot Description

## Useful commands:
```Shell
// Publish Ros
ros2 topic pub /pzb_gz_drive geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

// Publish Gazebo
gz topic -t /pzb_gz_drive -m gz.msgs.Twist -p 'linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}'
```
