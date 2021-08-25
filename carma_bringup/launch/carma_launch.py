# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, LogInfo, SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    bringup_dir = get_package_share_directory('carma_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    show_image = LaunchConfiguration('show_image')

    # TODO: Make using separate xterms an input argument
    # term_prefix = "xterm -fa 'Monospace' -fs 10 -geometry 120x30 -hold -e"
    term_prefix = ''

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to all nodes')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'carma_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the CARMA stack')

    declare_show_img_cmd = DeclareLaunchArgument(
        'show_image', default_value='false',
        description='Show image in camera client if true')

    # Composable node container for the Perception Subsystem nodes
    perception_container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='camera_driver',
                name='camera_driver',
                plugin='camera_driver::CameraDriver',
                extra_arguments=[{'use_intra_process_comms': True}]
                ),
            ComposableNode(
                package='camera_driver_client',
                name='camera_driver_client',
                plugin='camera_driver_client::CameraDriverClient',
                extra_arguments=[{'use_intra_process_comms': True}],
                parameters=[{'show_image': show_image}]
                ),
            ComposableNode(
                package='carma_delphi_srr2_driver',
                name='carma_delphi_srr2_driver',
                plugin='carma_delphi_srr2_driver::CarmaDelphiSrr2Driver',
                extra_arguments=[{'use_intra_process_comms': True}]
                ),
            ComposableNode(
                package='carma_velodyne_lidar_driver',
                name='carma_velodyne_lidar_driver',
                plugin='carma_velodyne_lidar_driver::CarmaVelodyneLidarDriver',
                extra_arguments=[{'use_intra_process_comms': True}]
                ),
            ],
        on_exit=[LogInfo(msg='perception_container')],
        output='screen',
        prefix=term_prefix,
        respawn='true'
        )

    # Localization Subsystem
    dead_reckoner = Node(
        package='dead_reckoner',
        name='dead_reckoner',
        executable='dead_reckoner',
        output='screen',
        prefix=term_prefix,
        respawn='true'
        )

    ekf_localizer = Node(
        package='ekf_localizer',
        name='ekf_localizer',
        executable='ekf_localizer',
        output='screen',
        prefix=term_prefix,
        respawn='true'
        )

    # Static Transform Publisher
    transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'camera']
        )

    localization_health_monitor = Node(
        package='localization_health_monitor',
        name='localization_health_monitor',
        executable='localization_health_monitor',
        output='screen',
        prefix=term_prefix,
        parameters=[
            {'auto_initialization_timeout': 3000},
            {'fitness_score_degraded_threshold': 20.0},
            {'fitness_score_fault_threshold': 100000.0},
            {'gnss_only_operation_timeout': 20000},
            {'ndt_frequency_degraded_threshold': 8.0},
            {'ndt_frequency_fault_threshold': 0.01}
            ],
        respawn='true'
        )

    # The system controller manages the lifecycle nodes
    carma_system_controller = Node(
        package='system_controller',
        name='carma_system_controller',
        executable='system_controller',
        output='screen',
        prefix=term_prefix,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': [
                'camera_driver',
                'camera_driver_client',
                'carma_delphi_srr2_driver',
                'carma_velodyne_lidar_driver',
                'dead_reckoner',
                'ekf_localizer',
                'localization_health_monitor'
                ]}
            ]
        )

    # Create the launch description
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_show_img_cmd)

    # Add the actions to launch the carma subsystems
    ld.add_action(perception_container)
    ld.add_action(dead_reckoner)
    ld.add_action(ekf_localizer)
    ld.add_action(localization_health_monitor)
    ld.add_action(transform_publisher_node)
    ld.add_action(carma_system_controller)
    return ld
