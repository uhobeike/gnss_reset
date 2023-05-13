# SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
# SPDX-License-Identifier: Apache-2.0


import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    gnss_reset_dir = get_package_share_directory('gnss_reset')
    gnss_reset_launch_dir = os.path.join(
        gnss_reset_dir, 'launch')
    rosbag_dir = os.path.join(
        gnss_reset_dir.replace('install/gnss_reset/share', 'src'), 'config', 'rosbag', 'map_with_gnss')

    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Set "true" to launch rviz.')

    embed_gnss2map_node = Node(
        name="embed_gnss2map_node",
        package='gnss_reset',
        executable='embed_gnss2map',
        parameters=[{'output_rosbag_path': rosbag_dir}],
        output='screen',
    )

    map_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gnss_reset_launch_dir,
                         'map_server.launch.py')
        ),
        launch_arguments={
        }.items()
    )

    rviz_config_file = os.path.join(
        gnss_reset_dir, 'config', 'rviz', 'gnss_reset.rviz')
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz))

    ld = LaunchDescription()

    ld.add_action(declare_use_rviz)

    ld.add_action(embed_gnss2map_node)
    ld.add_action(map_server)
    ld.add_action(rviz2_node)

    return ld
