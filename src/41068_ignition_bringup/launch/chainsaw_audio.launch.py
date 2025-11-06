#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('audio_topic',       default_value='/microphone/audio'),
        DeclareLaunchArgument('sample_rate',       default_value='16000'),
        DeclareLaunchArgument('frame_ms',          default_value='500'),
        DeclareLaunchArgument('hop_ms',            default_value='250'),
        DeclareLaunchArgument('chainsaw_low_hz',   default_value='90.0'),
        DeclareLaunchArgument('chainsaw_high_hz',  default_value='400.0'),
        DeclareLaunchArgument('confidence_thresh', default_value='0.55'),
        DeclareLaunchArgument('decision_window',   default_value='5'),
        DeclareLaunchArgument('log_period_sec',    default_value='1.0'),

        Node(
            package='41068_ignition_bringup',
            executable='chainsaw_detector_node.py',   # using installed script
            name='chainsaw_detector',
            output='screen',
            # makes launch call the script with python3 (avoids Exec format issues)
            prefix=['python3', ' '],
            #respawn=True,
            parameters=[{
                'audio_topic':       LaunchConfiguration('audio_topic'),
                'sample_rate':       LaunchConfiguration('sample_rate'),
                'frame_ms':          LaunchConfiguration('frame_ms'),
                'hop_ms':            LaunchConfiguration('hop_ms'),
                'chainsaw_low_hz':   LaunchConfiguration('chainsaw_low_hz'),
                'chainsaw_high_hz':  LaunchConfiguration('chainsaw_high_hz'),
                'confidence_thresh': LaunchConfiguration('confidence_thresh'),
                'decision_window':   LaunchConfiguration('decision_window'),
                'log_period_sec':    LaunchConfiguration('log_period_sec'),
            }],
        )
    ])
