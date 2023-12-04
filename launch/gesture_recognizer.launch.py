from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    image_topic = LaunchConfiguration('image_topic')
    image_topic_dec = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw/uncompressed',
        description='The name of the input image topic.')


    gesture_recognizer_node = Node(
            package='gesture_recognizer',
            executable='gesture_recognizer_node',
            # parameters=[params_file, {'tuning_mode': tune_detection}],
            remappings=[('/image_in',image_topic)],
            # condition=UnlessCondition(follow_only)
         )

    return LaunchDescription([
        # params_file_dec,
        # detect_only_dec,
        # follow_only_dec,
        # tune_detection_dec,
        # use_sim_time_dec,
        image_topic_dec,
        # cmd_vel_topic_dec,
        # enable_3d_tracker_dec,
        gesture_recognizer_node,
        # detect_3d_node,
        # follow_node,    
    ])