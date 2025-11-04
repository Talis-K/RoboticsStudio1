#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('audio_topic', default_value='/microphone/audio'),
        DeclareLaunchArgument('sample_rate', default_value='16000'),
        Node(
            package='41068_ignition_bringup',
            executable='chainsaw_detector_node.py',
            name='chainsaw_detector',
            output='screen',
            parameters=[{
                'audio_topic': LaunchConfiguration('audio_topic'),
                'sample_rate': LaunchConfiguration('sample_rate'),
            }]
        )
    ])
