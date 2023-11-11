# ROS2 Autonomous Robot Project

Trying to model a RC-car with Ackermann steering for simulation with Gazebo and ROS2 Humble.

Goal is to control a RC-car with Lidar and Depth Sensor for autonomous driving and navigation using SLAM-algorithms provided by ROS2. 

This is a work-in-progress. 

To build and source:
```bash
colcon build --symlink-install 
source install/setup.bash
```

To launch simple sim in gazebo with diff_drive controller and lidar 
```bash
ros2 launch my_bot launch_sim.launch.py
```

To control messages are published on /cmd_vel topic and can be controlled with
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

To launch rviz2 and gazebo with Ackermann Car Steering controller
```bash
ros2 launch my_bot bringup.launch.py
```

Inspect topics with 
```bash
rqt-graph
```