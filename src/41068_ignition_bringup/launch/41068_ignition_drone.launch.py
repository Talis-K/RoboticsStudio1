from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition



def generate_launch_description():

    ld = LaunchDescription()

    # Get paths to directories
    pkg_path = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg_path,
                                       'config'])

    # Additional command line arguments
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(use_sim_time_launch_arg)
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Flag to launch RViz'
    )
    ld.add_action(rviz_launch_arg)
    nav2_launch_arg = DeclareLaunchArgument(
        'nav2',
        default_value='True',
        description='Flag to launch Nav2'
    )
    ld.add_action(nav2_launch_arg)



    image_topic_launch_arg = DeclareLaunchArgument('image_topic', default_value='/camera/image')
    scan_topic_launch_arg  = DeclareLaunchArgument('scan_topic',  default_value='/scan')
    cloud_topic_launch_arg = DeclareLaunchArgument('cloud_topic', default_value='')  
    odom_topic_launch_arg  = DeclareLaunchArgument('odom_topic',  default_value='/odom')
    estop_topic_arg       = DeclareLaunchArgument('estop_topic',       default_value='/e_stop')
    max_altitude_arg      = DeclareLaunchArgument('max_altitude',      default_value='10.0')

    battery_topic_arg     = DeclareLaunchArgument('battery_topic',     default_value='/battery')
    gps_topic_arg         = DeclareLaunchArgument('gps_topic',         default_value='/gps/fix')
    imu_topic_arg         = DeclareLaunchArgument('imu_topic',         default_value='/imu')
    flight_mode_topic_arg = DeclareLaunchArgument('flight_mode_topic', default_value='/flight_mode')

    detections_topic_arg  = DeclareLaunchArgument('detections_topic',  default_value='/trees/cut')

    

    chainsaw_status_topic_arg  = DeclareLaunchArgument(
    'chainsaw_status_topic',  default_value='/audio/chainsaw/status',
    description='String status from chainsaw detector'
    )
    chainsaw_metrics_topic_arg = DeclareLaunchArgument(
        'chainsaw_metrics_topic', default_value='/audio/chainsaw/metrics',
        description='Float32MultiArray [class_id,conf,f0_hz,band_power]'
    )
    ld.add_action(chainsaw_status_topic_arg)
    ld.add_action(chainsaw_metrics_topic_arg)


    for a in [image_topic_launch_arg, scan_topic_launch_arg, cloud_topic_launch_arg, odom_topic_launch_arg, estop_topic_arg,
              max_altitude_arg, battery_topic_arg, gps_topic_arg, imu_topic_arg, flight_mode_topic_arg,detections_topic_arg]:
        ld.add_action(a)

    use_sim_time = LaunchConfiguration('use_sim_time')
    image_topic  = LaunchConfiguration('image_topic')
    scan_topic   = LaunchConfiguration('scan_topic')
    cloud_topic  = LaunchConfiguration('cloud_topic')
    ld.add_action(odom_topic_launch_arg)
    ld.add_action(estop_topic_arg)
    ld.add_action(max_altitude_arg)
    ld.add_action(battery_topic_arg)
    ld.add_action(gps_topic_arg)
    ld.add_action(imu_topic_arg)
    ld.add_action(flight_mode_topic_arg)
    ld.add_action(detections_topic_arg)
    ld.add_action(DeclareLaunchArgument('baro_topic', default_value='/baro'))
    ld.add_action(DeclareLaunchArgument('temperature_topic', default_value='/temperature'))
    ld.add_action(DeclareLaunchArgument('altitude_mode', default_value='auto'))
    ld.add_action(DeclareLaunchArgument('altitude_topic', default_value='')) 
    enable_audio_arg   = DeclareLaunchArgument('enable_audio', default_value='True')
    audio_topic_arg    = DeclareLaunchArgument('audio_topic',  default_value='/microphone/audio')
    audio_fs_arg       = DeclareLaunchArgument('audio_fs',     default_value='16000')
    audio_block_arg    = DeclareLaunchArgument('audio_block',  default_value='2048')
    audio_switch_arg   = DeclareLaunchArgument('audio_switch_period_s', default_value='5.0')
    ld.add_action(enable_audio_arg)
    ld.add_action(audio_topic_arg)
    ld.add_action(audio_fs_arg)
    ld.add_action(audio_block_arg)
    ld.add_action(audio_switch_arg)
    ld.add_action(DeclareLaunchArgument('waypoints_path_topic', default_value='/mission/waypoints_path'))
    ld.add_action(DeclareLaunchArgument('waypoints_array_topic', default_value='/mission/waypoints'))
   
    mission_cmd_topic_arg = DeclareLaunchArgument(
        'mission_cmd_topic',
        default_value='/mission/cmd',
        description='String command topic the GUI publishes and Mission subscribes to'
    )
    ld.add_action(mission_cmd_topic_arg)

    # after you add_action(mission_cmd_topic_arg)
    mission_cmd_topic = LaunchConfiguration('mission_cmd_topic')
    enable_mission_arg = DeclareLaunchArgument('enable_mission', default_value='True')
    ld.add_action(enable_mission_arg)


    # Load robot_description and start robot_state_publisher
    robot_description_content = ParameterValue(
        Command(['xacro ',
                 PathJoinSubstitution([pkg_path,
                                       'urdf_drone',
                                       'parrot.urdf.xacro'])]),
        value_type=str)
    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description_content,
                                          'use_sim_time': use_sim_time
                                      }])
    ld.add_action(robot_state_publisher_node)

    # Publish odom -> base_link transform **using robot_localization**
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[PathJoinSubstitution([config_path,
                                          'robot_localization.yaml']),
                    {'use_sim_time': use_sim_time}]
    )
    ld.add_action(robot_localization_node)

    # Start Gazebo to simulate the robot in the chosen world
    world_launch_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_trees',
        description='Which world to load',
        choices=['simple_trees', 'large_demo']
    )
    ld.add_action(world_launch_arg)
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'),
                             'launch', 'ign_gazebo.launch.py']),
        launch_arguments={
            'ign_args': [PathJoinSubstitution([pkg_path,
                                               'worlds',
                                               [LaunchConfiguration('world'), '.sdf']]),
                         ' -r']}.items()
    )
    ld.add_action(gazebo)

    # Spawn robot in Gazebo
    robot_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-topic', '/robot_description', '-z', '0.4'] # z is height above ground
    )
    ld.add_action(robot_spawner)

    # Bridge topics between gazebo and ROS2
    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': PathJoinSubstitution([config_path,
                                                          'gazebo_bridge.yaml']),
                    'use_sim_time': use_sim_time}]
    )
    ld.add_action(gazebo_bridge)

    # rviz2 visualises data
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_path,
                                               '41068.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    ld.add_action(rviz_node)

    # Nav2 enables mapping and waypoint following
    nav2 = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_path,
                              'launch',
                              '41068_navigation.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(nav2)

    gui_node = Node(
        package='41068_ignition_bringup',
        executable='gui_panel.py',   
        name='gui_panel',
        output='screen',
        parameters=[{
            'image_topic': image_topic,   
            'scan_topic':  scan_topic,   
            'cloud_topic': cloud_topic,  
            'odom_topic':   LaunchConfiguration('odom_topic'),
            'estop_topic':       LaunchConfiguration('estop_topic'),
            'max_altitude':      LaunchConfiguration('max_altitude'),

            'battery_topic':     LaunchConfiguration('battery_topic'),
            'gps_topic':         LaunchConfiguration('gps_topic'),
            'imu_topic':         LaunchConfiguration('imu_topic'),
            'flight_mode_topic': LaunchConfiguration('flight_mode_topic'),
            'detections_topic':  LaunchConfiguration('detections_topic'),
            'baro_topic':        LaunchConfiguration('baro_topic'),
            'temperature_topic': LaunchConfiguration('temperature_topic'),
            'altitude_mode':     LaunchConfiguration('altitude_mode'),
            'altitude_topic':    LaunchConfiguration('altitude_topic'),
            'chainsaw_status_topic':  LaunchConfiguration('chainsaw_status_topic'),
            'chainsaw_metrics_topic': LaunchConfiguration('chainsaw_metrics_topic'),
            'waypoints_path_topic': LaunchConfiguration('waypoints_path_topic'),
            'waypoints_array_topic': LaunchConfiguration('waypoints_array_topic'),
            'mission_cmd_topic':  LaunchConfiguration('mission_cmd_topic'),

        }]
    )
    ld.add_action(gui_node)


    audio_node = Node(
    package='41068_ignition_bringup',
    executable='test_audio_publisher.py',
    name='random_audio_pub',
    output='screen',
    parameters=[{
        'topic': LaunchConfiguration('audio_topic'),
        'fs':    LaunchConfiguration('audio_fs'),
        'block': LaunchConfiguration('audio_block'),
        'switch_period_s': LaunchConfiguration('audio_switch_period_s'),
    }],
    condition=IfCondition(LaunchConfiguration('enable_audio'))
    )
    ld.add_action(audio_node)



    # mission_node = Node(
    #     package='41068_ignition_bringup',
    #     executable='main.py',   # see note below about the executable name
    #     name='main',
    #     output='screen',
    #     parameters=[{
            
    #     }],
    #     condition=IfCondition(LaunchConfiguration('enable_mission'))
    #     )
    # ld.add_action(mission_node)


    mission_node = Node(
        package='41068_ignition_bringup',
        executable='main.py',   
        name='main',
        output='screen',
        parameters=[{
            'estop_topic':   LaunchConfiguration('estop_topic'),
            'cmd_topic':     mission_cmd_topic,               # <-- was hard-coded
            'state_topic':   '/mission/state',
            'wp_idx_topic':  '/mission/waypoint_index',
            'wp_total_topic':'/mission/waypoint_total',
            'progress_topic':'/mission/progress',
        }]
    )
    ld.add_action(mission_node)



    return ld
