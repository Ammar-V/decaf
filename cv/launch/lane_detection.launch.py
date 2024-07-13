from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node

import os

def generate_launch_description():

    package_name = 'cv'

    # Launch params
    image_input_topic = LaunchConfiguration('image_input_topic')
    declare_image_input_topic_cmd = DeclareLaunchArgument(
        'image_input_topic',
        default_value='/camera/depth/image_raw',
        description='Set the image input topic'
    )

    # Lane detection node
    detect_lanes = Node(
        package=package_name,
        executable='detect_lanes',
        remappings=[('/image_in', image_input_topic)]
    )


    return LaunchDescription([
        declare_image_input_topic_cmd,

        detect_lanes,
    ])



