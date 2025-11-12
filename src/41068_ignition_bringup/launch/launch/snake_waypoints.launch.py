from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/odom',
        description='Odometry topic feeding the pose printer'
    )
    waypoint_topic_arg = DeclareLaunchArgument(
        'waypoint_topic', default_value='snake_waypoints',
        description='Topic to publish snake waypoints'
    )

    node = Node(
        package='41068_ignition_bringup',
        executable='test_snake_waypoints.py',   # must match your console_script name
        name='waypoint_publisher',
        output='screen',
        remappings=[
            ('/odom', LaunchConfiguration('odom_topic')),
            ('snake_waypoints', LaunchConfiguration('waypoint_topic')),
        ],
    )

    return LaunchDescription([
        odom_topic_arg,
        waypoint_topic_arg,
        node
    ])
