import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description(): 

    package_name = "description"

    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Include the robot state publisher launch file
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': use_ros2_control}.items()
    )

    # Launch the twist_mux node
    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
    twist_out_topic = '/diff_cont/cmd_vel_unstamped' if use_ros2_control else '/cmd_vel'
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', twist_out_topic)],
        # TODO: If not using ros2_control, teleop_twist_keyboard will use /cmd_vel as output because gazebo_control expects
        # that as input. But, nav2 also uses that as output, so that might be a problem because nav2 and telop_twist_keyboard will clash!!
    )

    # Include the Gazebo launch file, provided by the gaebo ros_package
    gazebo_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
    )

    # Run the spawner node from the gazebo_ros package.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'decaf'],
                        output='screen')
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # Start the sensor fusion
    sensor_fusion_package_name = "scan_fusion"
    sensor_fusion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(sensor_fusion_package_name), 'launch', 'fused_sensor.launch.py'
        )])
    )

    # Start robot localization using an Extended Kalman filter
    robot_localization_file_path = os.path.join(get_package_share_directory(package_name), 'config', 'ekf.yaml')
    start_local_odom_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path, {'use_sim_time': True}],
        remappings=[("odometry/filtered", "odometry/local")])

    start_global_odom_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[robot_localization_file_path, {'use_sim_time': True}],
        remappings=[("odometry/filtered", "odometry/global")])
    
    start_navsat = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[robot_localization_file_path, {"use_sim_time": True}],
        remappings=[
            ("imu/data", "imu/data"), # sub
            ("gps/fix", "gps/fix"), # sub
            ("gps/filtered", "gps/filtered"), # pub
            ("odometry/gps", "odometry/gps"), # pub
            ("odometry/filtered", "odometry/global"), # sub
        ])


    # Launch them all
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'),
        
        rsp,
        twist_mux,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        sensor_fusion,

        start_local_odom_ekf,
        start_global_odom_ekf,
        start_navsat,
    ])