# Decaf

Learning how to make a mobile robot using ROS2 Humble.

## Commands

### Build and Source
```
cbs ; sws
```
These are aliases for <code>colcon build --symlink-install</code> and <code>source install/setup.bash</code>.

### Launch Rviz2
```
rviz2
```

### Launch Simulation in Gazebo
```
ros2 launch decaf launch_sim.launch.py world:=./src/decaf/worlds/obstacles.world
```

### Launch Teleop Twist Keyboard
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Launch Robot State Publisher (in simulation)
```
ros2 launch decaf rsp.launch.py use_sim_time:=true
```