import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    control_pkg = get_package_share_directory('a01_balance_control')
    controller_config = os.path.join(
        control_pkg,
        'config',
        'balance_controller.yaml',
    )

    controller = Node(
        package='a01_balance_control',
        executable='balance_controller',
        parameters=[controller_config],
        output='screen',
    )

    return LaunchDescription([controller])
