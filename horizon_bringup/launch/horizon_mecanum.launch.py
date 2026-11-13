from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch_ros.actions import Node
from launch.substitutions import AndSubstitution, Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    description_pkg = FindPackageShare('horizon_description')
    bringup_pkg = FindPackageShare('horizon_bringup')
    slam_pkg = FindPackageShare('slam_toolbox')
    nav_pkg = FindPackageShare('nav2_bringup')

    horizon_urdf_file = PathJoinSubstitution([description_pkg, 'urdf', 'horizon.urdf.xacro'])
    world_file = PathJoinSubstitution([bringup_pkg, 'worlds', 'room.sdf'])

    controllers_file = PathJoinSubstitution([bringup_pkg, 
                                             'config', 
                                             PythonExpression(["'horizon_ekf_controllers.yaml' if '", LaunchConfiguration('ekf'), "'.lower() == 'true' else 'horizon_mecanum_controllers.yaml'" ])])

    ros_gz_bridge_config_file = PathJoinSubstitution([bringup_pkg, 
                                                      'config', 
                                                      PythonExpression(["'horizon_bridge_ros2_control.yaml' if '", LaunchConfiguration('drive'), "'.lower() == 'ros2_control' else 'horizon_bridge_gazebo.yaml'"])])

    gz_launch_file = PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
    slam_online_async_launch_file = PathJoinSubstitution([slam_pkg, 'launch', 'online_async_launch.py'])
    slam_online_async_config_file = PathJoinSubstitution([bringup_pkg, 'config', 'mapper_params_online_async.yaml'])

    nav2_launch_file = PathJoinSubstitution([nav_pkg, "launch", "navigation_launch.py"])
    nav2_params_file = PathJoinSubstitution([bringup_pkg, 'config', 'nav2_params.yaml'])

    ekf_config_file = PathJoinSubstitution([bringup_pkg, 'config', 'horizon_ekf_params.yaml'])


    # Launch Arguments
    x_pos = DeclareLaunchArgument('x', default_value='0.0')
    y_pos = DeclareLaunchArgument('y', default_value='0.0')
    z_pos = DeclareLaunchArgument('z', default_value='0.1', 
                                  description='height to drop from')
    yaw_pos = DeclareLaunchArgument('yaw', default_value='0.0',
                                   description='Heading, radians')

    use_arm = DeclareLaunchArgument('use_arm', default_value="false")

    drive = DeclareLaunchArgument('drive', default_value="gz")
    system = DeclareLaunchArgument('system', default_value="mecanum")

    slam = DeclareLaunchArgument('slam', default_value="false")
    nav = DeclareLaunchArgument('nav', default_value='false')
    ekf = DeclareLaunchArgument('ekf', default_value='false')




    robot_description = ParameterValue(
        Command(['xacro ', horizon_urdf_file, 
                 ' drive:=', LaunchConfiguration('drive'), 
                 ' use_arm:=', LaunchConfiguration('use_arm'),
                 ' controllers_file:=', controllers_file,
                 ' system:=', LaunchConfiguration('system')]), 
                value_type=str)

    
    # Start Gazebo
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_file), 
        launch_arguments={
            'gz_args': ['-r ', world_file]
            }.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # Spawn Horizon in Gazebo
    spawn_horizon = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=['-topic', 'robot_description',
                   '-name', 'horizon_bot',
                   '-x', LaunchConfiguration('x'),
                   '-y', LaunchConfiguration('y'),
                   '-z', LaunchConfiguration('z'),
                   '-Y', LaunchConfiguration('yaw')]
    )

    # Controllers
    # Joint_state_broadcaster
    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['joint_state_broadcaster', '--switch-timeout', '30']
    )

    # JointGroupVelocity Controller
    spawn_joint_group_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['wheel_velocity_controller', '--switch-timeout', '30',
                   '--param-file', controllers_file]
    )

    spawn_diff_drive_controller = Node(
        package="controller_manager",
        executable='spawner',
        arguments=['diff_drive_controller', '--switch-timeout', '30',
                   '--param-file', controllers_file,
                   '--controller-ros-args', 
                   '-r /diff_drive_controller/cmd_vel:=/cmd_vel '
                   '-r /diff_drive_controller/odom:=/odom'
                   ]
    )

    spawn_mecanum_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['mecanum_drive_controller', '--switch-timeout', '30', 
                   '--param-file', controllers_file,
                   '--controller-ros-args', 
                   '-r /mecanum_drive_controller/tf_odometry:=/tf '
                   '-r /mecanum_drive_controller/reference:=/cmd_vel '
                   '-r /mecanum_drive_controller/odometry:=/odom'
                  ]
    )

    # Removed the cmd_vel remapping from here so I could use twist stamped on cmd_vel instead.
    # Kept the full arguments code here as I want to remember that the -rs are not separated by a comma
    # arguments=['mecanum_drive_controller', '--switch-timeout', '30', 
    #         '--controller-ros-args', 
    #         '-r /mecanum_drive_controller/tf_odometry:=/tf '
    #         '-r /mecanum_drive_controller/reference:=/cmd_vel']


    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            'config_file': ros_gz_bridge_config_file,
            'use_sim_time': True,
        }]
    )


    launch_joint_state_controller = RegisterEventHandler(
        OnProcessExit(target_action=spawn_horizon, on_exit=[spawn_jsb]),
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('drive'), "' == 'ros2_control'"
            ])
        )
    )

    launch_mecanum_controller = RegisterEventHandler(
        OnProcessExit(target_action=spawn_jsb, on_exit=[spawn_mecanum_controller]),
        condition=IfCondition(
            AndSubstitution(
                PythonExpression([
            "'", LaunchConfiguration('system'), "' == 'mecanum'"]),

                PythonExpression([
            "'", LaunchConfiguration('drive'), "' == 'ros2_control'"])
                )
            )
    )   


    launch_controllers_diff = RegisterEventHandler(
        OnProcessExit(target_action=spawn_jsb, on_exit=[spawn_diff_drive_controller]),
        condition=IfCondition(
            AndSubstitution(
                PythonExpression([
            "'", LaunchConfiguration('system'), "' == 'diff'"]),

                PythonExpression([
            "'", LaunchConfiguration('drive'), "' == 'ros2_control'"])
                )
            )
    )


    # SLAM
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_online_async_launch_file),
        launch_arguments={
            'slam_params_file': slam_online_async_config_file,
            'use_sim_time': 'true'
        }.items(),
    )

    # Confirming that odom is alive before launching slam
    odom_topic = PythonExpression([
        "'odometry/filtered' if '", LaunchConfiguration('ekf'), "'.lower() == 'true' else '/odom'"
    ])

    wait_for_odom = ExecuteProcess(
        cmd=['ros2', 'topic', 'echo', '--once', odom_topic, 'nav_msgs/msg/Odometry'],
        output='log',
        condition=IfCondition(LaunchConfiguration('slam'))
    )

    launch_slam = RegisterEventHandler(
        OnProcessExit(target_action=wait_for_odom, on_exit=[slam_toolbox]),
        condition=IfCondition(LaunchConfiguration('slam'))
    )

    # NAV2
    nav_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file
        }.items(),
    )

    # wait for map topic from SLAM
    map_topic = PythonExpression([
        "'/map'"
    ])

    wait_for_map = ExecuteProcess(
        cmd=['ros2', 'topic', 'echo', '--once', map_topic, 'nav_msgs/msg/OccupancyGrid'],
        output='log',
        condition=IfCondition(LaunchConfiguration('nav'))
    )

    launch_nav = RegisterEventHandler(
        OnProcessExit(target_action=wait_for_map, on_exit=[nav_bringup]),
        condition=IfCondition(LaunchConfiguration('nav'))
    )


    ekf_node = Node(
        package="robot_localization",
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('ekf'))
    )

    twist_stamper = Node(
        package="twist_stamper",
        executable="twist_stamper",
        parameters=[{
            'frame_id': 'base_footprint',
            'use_sim_time': True
        }],
        remappings=[
            ('/cmd_vel_in', '/cmd_vel'),
            ('/cmd_vel_out', '/mecanum_drive_controller/reference')
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([bringup_pkg, 'config', 'sim.rviz'])],
        parameters=[{'use_sim_time': True}],
    )


    return LaunchDescription([
        x_pos, y_pos, z_pos, yaw_pos, use_arm, drive, system, slam, nav, ekf,
        gz_launch,
        robot_state_publisher,
        spawn_horizon,
        ros_gz_bridge,
        launch_joint_state_controller,
        launch_mecanum_controller,
        launch_controllers_diff,
        wait_for_odom,
        launch_slam,
        wait_for_map,
        launch_nav,
        ekf_node,
        rviz_node,
        # twist_stamper
    ])