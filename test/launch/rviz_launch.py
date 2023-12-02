# Author: lei.zeng@tu-dortmund.de

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('homing_local_planner')

    rviz_config_file = LaunchConfiguration('rviz_config')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            pkg_dir, 'test/cfg', 'rviz_navigation.rviz'),
        description='Full path to the RVIZ config file to use')

    # Launch rviz
    rivz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')

    return LaunchDescription(
        [
            rivz_node
        ]
    )
