#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- Arguments (topics) ---
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/odom',
        description='Odometry topic for start pose if use_odom_start is True'
    )
    object_topic_arg = DeclareLaunchArgument(
        'object_topic', default_value='/object_geometry',
        description='Topic providing obstacle centre (x,y) and radius (z)'
    )
    path_topic_arg = DeclareLaunchArgument(
        'path_topic', default_value='/bypass_path',
        description='Output path topic'
    )

    # --- Arguments (params) ---
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='True')
    use_odom_start_arg = DeclareLaunchArgument('use_odom_start', default_value='True')
    start_x_arg = DeclareLaunchArgument('start_x', default_value='0.0')
    start_y_arg = DeclareLaunchArgument('start_y', default_value='0.0')
    goal_x_arg  = DeclareLaunchArgument('goal_x',  default_value='5.0')
    goal_y_arg  = DeclareLaunchArgument('goal_y',  default_value='0.0')
    frame_id_arg = DeclareLaunchArgument('frame_id', default_value='map')

    bypass_node = Node(
        package='41068_ignition_bringup',
        # Use the one that matches how you installed it:
        executable='obstacle_bypass_planner.py',   # if installed via CMake install(PROGRAMS)
        # executable='obstacle_bypass_planner',    # if installed via setup.py console_scripts
        name='obstacle_bypass_planner',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_odom_start': LaunchConfiguration('use_odom_start'),
            'start_x': LaunchConfiguration('start_x'),
            'start_y': LaunchConfiguration('start_y'),
            'goal_x': LaunchConfiguration('goal_x'),
            'goal_y': LaunchConfiguration('goal_y'),
            'frame_id': LaunchConfiguration('frame_id'),
        }],
        remappings=[
            ('/odom', LaunchConfiguration('odom_topic')),
            ('/object_geometry', LaunchConfiguration('object_topic')),
            ('/bypass_path', LaunchConfiguration('path_topic')),
        ],
        respawn=True
    )

    return LaunchDescription([
        odom_topic_arg, object_topic_arg, path_topic_arg,
        use_sim_time_arg, use_odom_start_arg,
        start_x_arg, start_y_arg, goal_x_arg, goal_y_arg, frame_id_arg,
        bypass_node
    ])
