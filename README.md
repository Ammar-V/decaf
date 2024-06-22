# Decaf

Learning how to make a mobile robot using ROS2 Humble.

## General Commands

### Build and Source
```
cbs ; sws
```
These are aliases for <code>colcon build --symlink-install</code> and <code>source install/setup.bash</code>.

### Launch Rviz2
```
rviz2 -d ./src/decaf/config/main.rviz
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
```

## ros2_control Commands

### List Hardware Interfaces
```
ros2 control list_hardware_interfaces
```

### Starting ros2_control controllers and broadcasters
```
ros2 run controller_manager spawner diff_cont
ros2 run controller_manager spawner joint_broad
```

### Launch Teleop Twist Keyboard 
With ros2_control controllers as input
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_teleop
```

## SLAM Commands (with SLAM Toolbox)

The following is only needed to be done once (it's already in the repo so don't need to run it). It copies over the config file from the package to a local directory, making it easier to edit.
```
cp /opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml ~/decaf_ws/src/decaf/config/
```

### Run SLAM
```
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/decaf/config/mapper_params_online_async.yaml use_sim_time:=true
```
The `online` means that it is using live data and `async` refers to the fact that it will use the latest perception data.

In `mapper_params_online_async.yaml`:
- `mode: map` will generate a map live and localize within it
- `mode: localization` will localize within a given map
  - Must set `map_file_name: /home/ammarvora/decaf_ws/<seralized_map_name>`

### Run Adaptive Monte Carlo Localization (AMCL) with a saved map

#### 1. Launch Gazebo and Rviz2
    - Set the fixed frame to `map` by force
    - Set the map's durability policy to `volatile`

#### 2. Run the map_server to publish the saved map
```
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/ammarvora/decaf_ws/my_map_save.yaml -p use_sim_time:=true
```
#### 3. Run the lifecycle bringup for map_server
```
ros2 run nav2_util lifecycle_bringup map_server
```
This will publish the map and it should show up in Rviz2

#### 4. Run AMCL
```
ros2 run nav2_amcl amcl --ros-args -p use_sim_time:=true
```

#### 5. Run the lifecycle bringup for AMCL
```
ros2 run nav2_util lifecycle_bringup amcl
```

#### 6. Set a 2D Pose Estimate in Rviz2


#### Alternative to steps 3-5
```
ros2 launch decaf localization_launch.py map:=./my_map_save.yaml use_sim_time:=true 
```

## Nav2 Commands

Launch mapping before launching nav2.
```
ros2 launch decaf navigation_launch.py use_sim_time:=true
```

### Running twist_mux (not necessary, launch_sim.launch.py takes care)
```
ros2 run twist_mux twist_mux --ros-args --params-file ./src/decaf/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped
```
