from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ld = LaunchDescription()

    # Get paths to directories
    pkg_path = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg_path, 'config'])

    # ---------------- Launch arguments ----------------
    use_sim_time_launch_arg = DeclareLaunchArgument('use_sim_time', default_value='True')
    rviz_launch_arg        = DeclareLaunchArgument('rviz',        default_value='False')
    nav2_launch_arg        = DeclareLaunchArgument('nav2',        default_value='True')
    world_launch_arg       = DeclareLaunchArgument('world',       default_value='simple_trees',
                                                   choices=['simple_trees', 'large_demo'])
    # NEW: topics for GUI
    image_topic_launch_arg = DeclareLaunchArgument('image_topic', default_value='/camera/image')
    scan_topic_launch_arg  = DeclareLaunchArgument('scan_topic',  default_value='/scan')
    cloud_topic_launch_arg = DeclareLaunchArgument('cloud_topic', default_value='')  # optional

    for a in [use_sim_time_launch_arg, rviz_launch_arg, nav2_launch_arg,
              world_launch_arg, image_topic_launch_arg, scan_topic_launch_arg, cloud_topic_launch_arg]:
        ld.add_action(a)

    use_sim_time = LaunchConfiguration('use_sim_time')
    image_topic  = LaunchConfiguration('image_topic')
    scan_topic   = LaunchConfiguration('scan_topic')
    cloud_topic  = LaunchConfiguration('cloud_topic')

    # ---------------- Robot description + state publisher ----------------
    robot_description_content = ParameterValue(
        Command(['xacro ',
                 PathJoinSubstitution([pkg_path, 'urdf_drone', 'parrot.urdf.xacro'])]),
        value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': use_sim_time}]
    )
    ld.add_action(robot_state_publisher_node)

    # ---------------- Localization ----------------
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[PathJoinSubstitution([config_path, 'robot_localization.yaml']),
                    {'use_sim_time': use_sim_time}]
    )
    ld.add_action(robot_localization_node)

    # ---------------- Gazebo world ----------------
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py']),
        launch_arguments={
            'ign_args': [PathJoinSubstitution([pkg_path, 'worlds', [LaunchConfiguration('world'), '.sdf']]),
                         ' -r']
        }.items()
    )
    ld.add_action(gazebo)

    # ---------------- Spawn robot ----------------
    robot_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-topic', '/robot_description', '-z', '0.4']
    )
    ld.add_action(robot_spawner)

    # ---------------- Bridge topics ----------------
    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': PathJoinSubstitution([config_path, 'gazebo_bridge.yaml']),
                    'use_sim_time': use_sim_time}]
    )
    ld.add_action(gazebo_bridge)

    # ---------------- RViz ----------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_path, '41068.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    ld.add_action(rviz_node)

    # ---------------- GUI panel ----------------
    gui_node = Node(
        package='41068_ignition_bringup',
        executable='gui_panel.py',   # keep as-is since this works in your env
        name='gui_panel',
        output='screen',
        parameters=[{
            'image_topic': image_topic,   # defaults to /camera/image
            'scan_topic':  scan_topic,    # defaults to /scan
            'cloud_topic': cloud_topic,   # set to a PointCloud2 topic if you want to use it
            'estop_topic': '/e_stop',
        }]
        # , prefix='xterm -e'
    )
    ld.add_action(gui_node)

    # ---------------- Nav2 ----------------
    nav2 = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_path, 'launch', '41068_navigation.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(nav2)

    return ld
