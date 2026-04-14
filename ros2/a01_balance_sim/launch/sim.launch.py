import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_pkg = get_package_share_directory('a01_balance_sim')
    control_pkg = get_package_share_directory('a01_balance_control')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    urdf_path = os.path.join(sim_pkg, 'urdf', 'a01_balance_sim.xacro')
    world_path = os.path.join(sim_pkg, 'worlds', 'empty.sdf')
    controller_config = os.path.join(
        control_pkg,
        'config',
        'balance_controller.yaml',
    )
    robot_description = xacro.process_file(urdf_path).toxml()
    gz_args = LaunchConfiguration('gz_args')
    spawn_z = LaunchConfiguration('spawn_z')
    ball_pos_ref = LaunchConfiguration('ball_pos_ref')
    ball_pos_kp = LaunchConfiguration('ball_pos_kp')
    ball_vel_kd = LaunchConfiguration('ball_vel_kd')
    track_angle_kp = LaunchConfiguration('track_angle_kp')
    track_rate_kd = LaunchConfiguration('track_rate_kd')
    max_track_angle = LaunchConfiguration('max_track_angle')
    max_track_velocity = LaunchConfiguration('max_track_velocity')
    command_alpha = LaunchConfiguration('command_alpha')
    max_cmd_step = LaunchConfiguration('max_cmd_step')
    ball_pos_alpha = LaunchConfiguration('ball_pos_alpha')
    ball_vel_alpha = LaunchConfiguration('ball_vel_alpha')
    track_angle_alpha = LaunchConfiguration('track_angle_alpha')
    track_rate_alpha = LaunchConfiguration('track_rate_alpha')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/world/default/model/a01_balance/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/model/a01_balance/joint/joint_track/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        remappings=[
            ('/world/default/model/a01_balance/joint_state', '/joint_states'),
        ],
        output='screen',
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        parameters=[{
            'name': 'a01_balance',
            'topic': 'robot_description',
            'x': 0.0,
            'y': 0.0,
            'z': spawn_z,
        }],
        output='screen',
    )

    controller = Node(
        package='a01_balance_control',
        executable='balance_controller',
        parameters=[{
            'use_sim_time': True,
        }, controller_config, {
            'ball_pos_ref': ball_pos_ref,
            'ball_pos_kp': ball_pos_kp,
            'ball_vel_kd': ball_vel_kd,
            'track_angle_kp': track_angle_kp,
            'track_rate_kd': track_rate_kd,
            'max_track_angle': max_track_angle,
            'max_track_velocity': max_track_velocity,
            'command_alpha': command_alpha,
            'max_cmd_step': max_cmd_step,
            'ball_pos_alpha': ball_pos_alpha,
            'ball_vel_alpha': ball_vel_alpha,
            'track_angle_alpha': track_angle_alpha,
            'track_rate_alpha': track_rate_alpha,
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gz_args',
            default_value=f'-r {world_path}',
            description='Arguments forwarded to gz sim',
        ),
        DeclareLaunchArgument('spawn_z', default_value='0.08'),
        DeclareLaunchArgument('ball_pos_ref', default_value='0.0'),
        DeclareLaunchArgument('ball_pos_kp', default_value='2.6'),
        DeclareLaunchArgument('ball_vel_kd', default_value='1.1'),
        DeclareLaunchArgument('track_angle_kp', default_value='8.0'),
        DeclareLaunchArgument('track_rate_kd', default_value='0.55'),
        DeclareLaunchArgument('max_track_angle', default_value='0.10'),
        DeclareLaunchArgument('max_track_velocity', default_value='2.2'),
        DeclareLaunchArgument('command_alpha', default_value='0.18'),
        DeclareLaunchArgument('max_cmd_step', default_value='0.20'),
        DeclareLaunchArgument('ball_pos_alpha', default_value='0.20'),
        DeclareLaunchArgument('ball_vel_alpha', default_value='0.18'),
        DeclareLaunchArgument('track_angle_alpha', default_value='0.20'),
        DeclareLaunchArgument('track_rate_alpha', default_value='0.16'),
        gz_sim,
        rsp,
        bridge,
        controller,
        TimerAction(period=2.0, actions=[spawn]),
    ])
