# Import the LaunchDescription class, which holds all actions to run at launch time.
from launch import LaunchDescription
# Import core launch actions for declaring args and including other launch files.
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# Enable conditional execution of actions (e.g., only launch RViz if flag is True).
from launch.conditions import IfCondition
# Substitutions allow strings/values to be computed at launch (e.g., from args, paths).
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
# ROS 2 action that starts a node process.
from launch_ros.actions import Node
# Lets us pass a computed/command-generated string (like xacro output) as a parameter.
from launch_ros.parameter_descriptions import ParameterValue
# Helper to locate a package's share/ directory at runtime.
from launch_ros.substitutions import FindPackageShare
# (Duplicate import retained exactly as provided) Conditional execution import.
from launch.conditions import IfCondition



def generate_launch_description():
    # Entry point required by the launch system. Must return a LaunchDescription.

    # Create the top-level description object that will collect all actions.
    ld = LaunchDescription()

    # Resolve package path for this project; used to build URIs to configs/URDF/worlds.
    pkg_path = FindPackageShare('41068_ignition_bringup')
    # Compose a path to the package's config directory using substitutions.
    config_path = PathJoinSubstitution([pkg_path,
                                       'config'])

    # ---- Launch arguments (CLI flags) --------------------------------------------------------
    # Enable/disable simulated time across all nodes (default True because we're in sim).
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )
    # Create a handle to read the value of use_sim_time later in this file.
    use_sim_time = LaunchConfiguration('use_sim_time')
    # Register the argument so it appears and is usable at runtime.
    ld.add_action(use_sim_time_launch_arg)
    # Whether to bring up RViz (visualization). Off by default to save resources.
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Flag to launch RViz'
    )
    # Register the RViz argument.
    ld.add_action(rviz_launch_arg)
    # Whether to launch the Nav2 stack (mapping/planning). On by default.
    nav2_launch_arg = DeclareLaunchArgument(
        'nav2',
        default_value='True',
        description='Flag to launch Nav2'
    )
    # Register the Nav2 argument.
    ld.add_action(nav2_launch_arg)



    # Topic name arguments for the GUI and other nodes (camera, LiDAR, etc.).
    image_topic_launch_arg = DeclareLaunchArgument('image_topic', default_value='/camera/image')
    scan_topic_launch_arg  = DeclareLaunchArgument('scan_topic',  default_value='/scan')
    # Point cloud is optional; empty by default.
    cloud_topic_launch_arg = DeclareLaunchArgument('cloud_topic', default_value='')  
    # Standard odometry topic.
    odom_topic_launch_arg  = DeclareLaunchArgument('odom_topic',  default_value='/odom')
    # Emergency stop topic for safety control.
    estop_topic_arg       = DeclareLaunchArgument('estop_topic',       default_value='/e_stop')
    # Upper bound on commanded altitude (enforced in GUI/mission logic).
    max_altitude_arg      = DeclareLaunchArgument('max_altitude',      default_value='10.0')
    # IMU telemetry topic name.
    imu_topic_arg         = DeclareLaunchArgument('imu_topic',         default_value='/imu')
    # Flight mode status topic (string/enum depending on your stack).
    flight_mode_topic_arg = DeclareLaunchArgument('flight_mode_topic', default_value='/flight_mode')
    # Tree detection events (for cut/marked trees, etc.) topic used by the GUI.
    detections_topic_arg  = DeclareLaunchArgument('detections_topic',  default_value='/trees/cut')
    # Re-bind local variables to launch-time values (used below in parameters).
    use_sim_time = LaunchConfiguration('use_sim_time')
    image_topic  = LaunchConfiguration('image_topic')
    scan_topic   = LaunchConfiguration('scan_topic')
    cloud_topic  = LaunchConfiguration('cloud_topic')
    # Toggle the example audio publisher (chainsaw demo). True by default.
    enable_audio_arg   = DeclareLaunchArgument('enable_audio', default_value='True')
    # Raw microphone topic (audio buffer).
    audio_topic_arg    = DeclareLaunchArgument('audio_topic',  default_value='/microphone/audio')
    # Sample rate for the audio generator.
    audio_fs_arg       = DeclareLaunchArgument('audio_fs',     default_value='16000')
    # Block size (samples per message) for the audio publisher.
    audio_block_arg    = DeclareLaunchArgument('audio_block',  default_value='2048')
    # How often the audio node switches classes/sources (demo).
    audio_switch_arg   = DeclareLaunchArgument('audio_switch_period_s', default_value='5.0')
    # Status string from audio classifier (e.g., chainsaw detected).
    chainsaw_status_topic_arg  = DeclareLaunchArgument(
    'chainsaw_status_topic',  default_value='/audio/chainsaw/status',
    description='String status from chainsaw detector'
    )
    # Classifier metrics (confidence, features) channel for analysis/debug.
    chainsaw_metrics_topic_arg = DeclareLaunchArgument(
        'chainsaw_metrics_topic', default_value='/audio/chainsaw/metrics',
        description='Float32MultiArray [class_id,conf,f0_hz,band_power]'
    )
    # Count of detected stumps (displayed on GUI).
    stump_count_topic_arg = DeclareLaunchArgument('stump_count_topic', default_value='/mission/stump_count')
    # Altitude mode selection (e.g., auto/manual). Consumed by GUI/mission code.
    ld.add_action(DeclareLaunchArgument('altitude_mode', default_value='auto'))
    # Optional input topic that provides altitude (if external source is used).
    ld.add_action(DeclareLaunchArgument('altitude_topic', default_value='')) 
    # Path visualization of waypoints for the GUI.
    ld.add_action(DeclareLaunchArgument('waypoints_path_topic', default_value='/mission/waypoints_path'))
    # Array of waypoints for the GUI/mission.
    ld.add_action(DeclareLaunchArgument('waypoints_array_topic', default_value='/mission/waypoints'))
    # Register stump count topic.
    ld.add_action(stump_count_topic_arg)
    # Register all the previously declared core telemetry arguments.
    ld.add_action(odom_topic_launch_arg)
    ld.add_action(estop_topic_arg)
    ld.add_action(max_altitude_arg)
    ld.add_action(imu_topic_arg)
    ld.add_action(flight_mode_topic_arg)
    ld.add_action(detections_topic_arg)
    # Register audio-related arguments so they can be read by the node.
    ld.add_action(enable_audio_arg)
    ld.add_action(audio_topic_arg)
    ld.add_action(audio_fs_arg)
    ld.add_action(audio_block_arg)
    ld.add_action(audio_switch_arg)
    # Register audio classifier status/metrics topics.
    ld.add_action(chainsaw_status_topic_arg)
    ld.add_action(chainsaw_metrics_topic_arg)

    # Add the image/scan/cloud/odom/etc. args in a loop (kept exactly as given).
    for a in [image_topic_launch_arg, scan_topic_launch_arg, cloud_topic_launch_arg, odom_topic_launch_arg, estop_topic_arg,
              max_altitude_arg, imu_topic_arg, flight_mode_topic_arg,detections_topic_arg]:
        # Register each of these arguments with the launch description.
        ld.add_action(a)
   
    # Topic that GUI will publish mission commands on; mission node subscribes to it.
    mission_cmd_topic_arg = DeclareLaunchArgument(
        'mission_cmd_topic',
        default_value='/mission/cmd',
        description='String command topic the GUI publishes and Mission subscribes to'
    )
    # Register the mission command topic argument.
    ld.add_action(mission_cmd_topic_arg)

    # GUI counters/telemetry topics for detected trees and people.
    tree_count_topic_arg   = DeclareLaunchArgument('tree_count_topic',   default_value='/mission/tree_count')
    people_count_topic_arg = DeclareLaunchArgument('people_count_topic', default_value='/mission/people_count')
    # Register these count topics.
    ld.add_action(tree_count_topic_arg)
    ld.add_action(people_count_topic_arg)


    # Read back the mission cmd topic value for use in node parameters below.
    mission_cmd_topic = LaunchConfiguration('mission_cmd_topic')
    # Master toggle to enable the mission node (kept True by default).
    enable_mission_arg = DeclareLaunchArgument('enable_mission', default_value='True')
    # Register the toggle.
    ld.add_action(enable_mission_arg)


    # ---- Robot description & TF pipeline -----------------------------------------------------
    # Build the URDF at launch using xacro, capturing the generated XML as a string parameter.
    robot_description_content = ParameterValue(
        Command(['xacro ',
                 PathJoinSubstitution([pkg_path,
                                       'urdf_drone',
                                       'parrot.urdf.xacro'])]),
        value_type=str)
    # Publish /tf using robot_state_publisher so the robot model is available to the ecosystem.
    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description_content,
                                          'use_sim_time': use_sim_time
                                      }])
    # Add the state publisher to be launched.
    ld.add_action(robot_state_publisher_node)

    # Run EKF (robot_localization) to fuse sensors into a filtered odometry frame.
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[PathJoinSubstitution([config_path,
                                          'robot_localization.yaml']),
                    {'use_sim_time': use_sim_time}]
    )
    # Add the EKF node to the launch description.
    ld.add_action(robot_localization_node)

    # ---- Gazebo (Ignition) simulator bringup -------------------------------------------------
    # Select which world SDF to load (choices limited to known world files).
    world_launch_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_trees',
        description='Which world to load',
        choices=['simple_trees', 'large_demo', 'simple_trees_copy']
    )
    # Register the world selection argument.
    ld.add_action(world_launch_arg)
    # Include the standard ros_ign_gazebo launch to start the simulator and load the chosen world.
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'),
                             'launch', 'ign_gazebo.launch.py']),
        launch_arguments={
            'ign_args': [PathJoinSubstitution([pkg_path,
                                               'worlds',
                                               [LaunchConfiguration('world'), '.sdf']]),
                         ' -r']}.items()
    )
    # Add the simulator include to the launch.
    ld.add_action(gazebo)

    # Spawn the robot entity into the world from the /robot_description topic.
    robot_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-topic', '/robot_description', '-z', '0.4'] 
    )
    # Add the entity spawner.
    ld.add_action(robot_spawner)

    # Bridge Gazebo transport topics to ROS 2 so nodes can subscribe/publish normally.
    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': PathJoinSubstitution([config_path,
                                                          'gazebo_bridge.yaml']),
                    'use_sim_time': use_sim_time}]
    )
    # Add the bridge node (configured via YAML for specific topics/types).
    ld.add_action(gazebo_bridge)

    # ---- Tooling / Visualization -------------------------------------------------------------
    # RViz2 client; loads a pre-configured RViz layout file when enabled.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_path,
                                               '41068.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    # Add RViz (conditionally runs if the 'rviz' arg is true).
    ld.add_action(rviz_node)

    # Nav2 stack (maps, plans, and follows waypoints) enabled by the 'nav2' flag.
    nav2 = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_path,
                              'launch',
                              '41068_navigation.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    # Add the Nav2 include (runs lifecycle bringup, planners, controllers, etc.).
    ld.add_action(nav2)

    # ---- GUI panel ---------------------------------------------------------------------------
    # Custom GUI that shows telemetry, plots, and lets you send mission commands.
    gui_node = Node(
        package='41068_ignition_bringup',
        executable='gui_panel.py',   
        name='gui_panel',
        output='screen',
        parameters=[{
            # Video image source (camera topic).
            'image_topic': image_topic,   
            # 2D laser scan source.
            'scan_topic':  scan_topic,   
            # Optional point cloud source.
            'cloud_topic': cloud_topic,  
            # Pose/velocity source for the vehicle.
            'odom_topic':   LaunchConfiguration('odom_topic'),
            # Emergency stop status/control topic.
            'estop_topic':       LaunchConfiguration('estop_topic'),
            # GUI clamp for altitude targets.
            'max_altitude':      LaunchConfiguration('max_altitude'),
            # IMU topic for attitude/accel data.
            'imu_topic':         LaunchConfiguration('imu_topic'),
            # Text/enum indicating PX4/ArduPilot/etc. state.
            'flight_mode_topic': LaunchConfiguration('flight_mode_topic'),
            # Detection events (e.g., tree cut markers).
            'detections_topic':  LaunchConfiguration('detections_topic'),
            # Altitude mode (auto/manual) influences GUI behavior.
            'altitude_mode':     LaunchConfiguration('altitude_mode'),
            # Optional altitude input topic (e.g., baro or fused).
            'altitude_topic':    LaunchConfiguration('altitude_topic'),
            # Audio classifier status for GUI banner/indicators.
            'chainsaw_status_topic':  LaunchConfiguration('chainsaw_status_topic'),
            # Audio classifier numeric metrics for plots/debug.
            'chainsaw_metrics_topic': LaunchConfiguration('chainsaw_metrics_topic'),
            # Polyline path of all waypoints (for map drawing).
            'waypoints_path_topic': LaunchConfiguration('waypoints_path_topic'),
            # Individual waypoints array (for list and selection).
            'waypoints_array_topic': LaunchConfiguration('waypoints_array_topic'),
            # Command channel the GUI publishes mission actions to.
            'mission_cmd_topic':  LaunchConfiguration('mission_cmd_topic'),
            # GUI counters for trees detected.
            'tree_count_topic':   LaunchConfiguration('tree_count_topic'),
            # GUI counters for people detected.
            'people_count_topic': LaunchConfiguration('people_count_topic'),
            # GUI counter for detected stumps.
            'stump_count_topic': LaunchConfiguration('stump_count_topic'),
        }]
    )
    # Add the GUI node to the launch.
    ld.add_action(gui_node)

<<<<<<< HEAD
=======

    path_node = Node(
        package='41068_ignition_bringup',
        executable='test_snake_waypoints.py',   # keep as-is since this works in your env
        name='path_planning',
        output='screen',
        parameters=[{
            'image_topic': image_topic,   # defaults to /camera/image
            'scan_topic':  scan_topic,    # defaults to /scan
            'cloud_topic': cloud_topic,   # set to a PointCloud2 topic if you want to use it
            'estop_topic': '/e_stop',
        }]
        # , prefix='xterm -e'
    )
    ld.add_action(path_node)

    # ---------------- Nav2 ----------------
    nav2 = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_path, 'launch', '41068_navigation.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(nav2)
>>>>>>> 772b88c224f695a13e319f46d00cfcf463f2dd54

    # Example/demo audio publisher that generates/alternates audio frames and topics.
    audio_node = Node(
    package='41068_ignition_bringup',
    executable='test_audio_publisher.py',
    name='random_audio_pub',
    output='screen',
    parameters=[{
        # Output audio topic.
        'topic': LaunchConfiguration('audio_topic'),
        # Sampling rate used by the publisher.
        'fs':    LaunchConfiguration('audio_fs'),
        # Block size per published message.
        'block': LaunchConfiguration('audio_block'),
        # Time interval for switching audio classes/sources.
        'switch_period_s': LaunchConfiguration('audio_switch_period_s'),
    }],
    # Only run if enable_audio is true.
    condition=IfCondition(LaunchConfiguration('enable_audio'))
    )
    # Add the audio demo node.
    ld.add_action(audio_node)


    # Mission node orchestrates behavior: reads GUI commands, reports state/progress.
    mission_node = Node(
        package='41068_ignition_bringup',
        executable='main.py',   
        name='main',
        output='screen',
        parameters=[{
            # E-stop topic subscription for safety gating.
            'estop_topic':   LaunchConfiguration('estop_topic'),
            # What command topic to listen to for mission directives.
            'cmd_topic':     mission_cmd_topic,              
            # Human-readable state string for GUI/logging.
            'state_topic':   '/mission/state',
            # Current waypoint index (int).
            'wp_idx_topic':  '/mission/waypoint_index',
            # Total waypoints (int).
            'wp_total_topic':'/mission/waypoint_total',
            # Overall progress percent (0-100).
            'progress_topic':'/mission/progress',
        }]
    )
    # Add the mission node to the launch.
    ld.add_action(mission_node)



    # Return the composed launch description to the ROS 2 launch system.
    return ld
