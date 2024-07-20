from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os

def generate_launch_description():

    cv_package_name = 'cv'
    pc_to_laser_package_name = 'pointcloud_to_laserscan'
    package_name = 'scan_fusion'

    lane_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(cv_package_name), 'launch', 'lane_detection.launch.py'
        )])
    )

    pc_to_laser = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(pc_to_laser_package_name), 'launch', 'sample_pointcloud_to_laserscan_launch.py'
        )])
    )

    # Fusion node
    fusion = Node(
        package=package_name,
        executable='fuse_scans',
    )


    return LaunchDescription([
       lane_detection,
       pc_to_laser,
       fusion,
    ])



