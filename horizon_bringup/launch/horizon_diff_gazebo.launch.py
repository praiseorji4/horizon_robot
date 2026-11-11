from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    description_pkg = FindPackageShare('horizon_description')
    bringup_pkg = FindPackageShare('horizon_bringup')

    x_pos = DeclareLaunchArgument('x', default_value='0.0')
    y_pos = DeclareLaunchArgument('y', default_value='0.0')
    z_pos = DeclareLaunchArgument('z', default_value='0.1', 
                                  description='height to drop from')
    yaw_pos = DeclareLaunchArgument('yaw', default_value='0.0',
                                   description='Heading, radians')

    use_arm = DeclareLaunchArgument('use_arm', default_value="false")

    drive = DeclareLaunchArgument('drive', default_value="")

    
    xacro_file = PathJoinSubstitution([description_pkg, 'urdf', 'horizon.urdf.xacro'])
    world_file = PathJoinSubstitution([bringup_pkg, 'worlds', 'room.sdf'])
    controllers_file = PathJoinSubstitution([bringup_pkg, 'config', 'controllers.yaml'])
    robot_description = ParameterValue(Command(['xacro ', xacro_file, ' use_arm:=', LaunchConfiguration('use_arm'),
                                         ' drive:=', LaunchConfiguration('drive'), ' controllers_file:=', controllers_file]), value_type=str)
    

    gz_launch_file = PathJoinSubstitution(
        [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
    )



    gazebo_launch_node = IncludeLaunchDescription(
        gz_launch_file, launch_arguments={'gz_args': ['-r ', world_file]}.items()
    )                               

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description,
             'use_sim_time': True}
        ],
    )

    robot_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'horizon_bot',
                   '-x', LaunchConfiguration('x'),
                   '-y', LaunchConfiguration('y'),
                   '-z', LaunchConfiguration('z'),
                   '-Y', LaunchConfiguration('yaw')],
    )

    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable='spawner',
        arguments=['joint_state_broadcaster', '--switch-timeout', '30']
    )

    spawn_diff_drive_controller = Node(
        package="controller_manager",
        executable='spawner',
        arguments=['diff_drive_controller', '--switch-timeout', '30']
    )

    launch_controllers = RegisterEventHandler(
        OnProcessExit(target_action=robot_spawner, on_exit=[spawn_joint_state_broadcaster, spawn_diff_drive_controller])
    )


    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {'config_file': PathJoinSubstitution([bringup_pkg, 'config', 'horizon_bridge_ros2_control.yaml']),
            'use_sim_time': True,}
        ]
    )


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([bringup_pkg, 'config', 'sim.rviz'])],
        parameters=[{'use_sim_time': True}],
    )

    # Using Arguments Version
    # ros_gz_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=['model/horizon_bot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
    #     remappings=[('/model/horizon_bot/cmd_vel', '/cmd_vel')],
    # )


    return LaunchDescription([
        use_arm, drive,
        gazebo_launch_node,
        robot_state_publisher_node,
        x_pos, y_pos, z_pos, yaw_pos,
        robot_spawner,
        launch_controllers,
        ros_gz_bridge, 
        rviz_node
    ])
