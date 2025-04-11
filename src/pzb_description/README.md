# Puzzlebot Description

## Rviz/Gazebo + Twist

For puzzlebot visualization in rviz.
```
ros2 launch pzb_description rviz_launch.py 
```
Launch gazebo.
```
ros2 launch pzb_description gz_launch.py

```
For state estimate (Need submodule TurboPlusPlus_ImplementacionRobotica/).
```
ros2 launch pzb_control pzb_control_launch.py
```
For controlling puzzlebot with `cmd_vel`.
```
ros2 run pzb_control pzb_teleop_low_level_node.py
```
## Troubleshooting
If gazebo runs with only a black screen uncomment `gz_ip` from LaunchDescription in `gz_launch.py`.
