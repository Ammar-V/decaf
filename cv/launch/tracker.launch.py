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
        default_value='/camera/image_raw',
        description='Set the image input topic'
    )

    # Object detection node
    detect_object = Node(
        package=package_name,
        executable='detect_object',
        remappings=[('/image_in', image_input_topic)]
    )

    # Object following node
    follow_object_params = os.path.join(get_package_share_directory(package_name), 'config', 'cv_params.yaml')
    follow_object = Node(
        package=package_name,
        executable='follow_object',
        parameters=[follow_object_params]
    )


    return LaunchDescription([
        declare_image_input_topic_cmd,

        detect_object,
        follow_object,
    ])



