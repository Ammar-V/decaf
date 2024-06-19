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

### View Image Topics
```
ros2 run rqt_image_view rqt_image_view
```

### Republish Images (compressed -> raw)
```
ros2 run image_transport republish compressed raw --ros-args -r in/compressed:=/camera/image_raw/compressed -r out:=/camera/image_raw/uncompressed
``