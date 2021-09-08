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

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction)
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.ros_adapters import get_ros_node

from cav_msgs.msg import SystemAlert

shutdown_published = False


def publish_shutdown(context):
    global shutdown_published
    if not shutdown_published:
        shutdown_published = True
        node = get_ros_node(context)

        alert = SystemAlert()
        alert.type = SystemAlert.SHUTDOWN
        alert.description = 'An example SHUTDOWN system alert, resulting from a launched process exiting'

        publisher = node.create_publisher(SystemAlert, '/system_alert', 10)
        publisher.publish(alert)


def generate_launch_description():

    # Create the launch configuration variables
    autostart = LaunchConfiguration('autostart')
    namespace = LaunchConfiguration('namespace')
    show_image = LaunchConfiguration('show_image')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Can set the CARMA_LAUNCH_PREFIX in the environment
    term_prefix = EnvironmentVariable('CARMA_LAUNCH_PREFIX', default_value=''),

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # Declare the launch arguments
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the CARMA stack')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_show_img_cmd = DeclareLaunchArgument(
        'show_image', default_value='false',
        description='Show image in camera client if true')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # The composable node container for the Perception Subsystem
    perception_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='perception_container',
        namespace=namespace,
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='camera_driver',
                name='camera_driver',
                namespace=namespace,
                plugin='camera_driver::CameraDriver',
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='camera_driver_client',
                name='camera_driver_client',
                namespace=namespace,
                plugin='camera_driver_client::CameraDriverClient',
                extra_arguments=[{'use_intra_process_comms': True}],
                parameters=[{'show_image': show_image}]
            ),
            ComposableNode(
                package='carma_delphi_srr2_driver',
                name='carma_delphi_srr2_driver',
                namespace=namespace,
                plugin='carma_delphi_srr2_driver::CarmaDelphiSrr2Driver',
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='carma_velodyne_lidar_driver',
                name='carma_velodyne_lidar_driver',
                namespace=namespace,
                plugin='carma_velodyne_lidar_driver::CarmaVelodyneLidarDriver',
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='screen',
        prefix=term_prefix,
        on_exit=[OpaqueFunction(function=publish_shutdown)]
    )

    # Nodes in the Localization Subsystem
    dead_reckoner = Node(
        package='dead_reckoner',
        name='dead_reckoner',
        namespace=namespace,
        executable='dead_reckoner',
        output='screen',
        prefix=term_prefix,
        on_exit=[OpaqueFunction(function=publish_shutdown)]
    )

    ekf_localizer = Node(
        package='ekf_localizer',
        name='ekf_localizer',
        namespace=namespace,
        executable='ekf_localizer',
        output='screen',
        prefix=term_prefix,
        on_exit=[OpaqueFunction(function=publish_shutdown)]
    )

    # Static transform publisher
    transform_publisher_node = Node(
        package='tf2_ros',
        name='static_transform_publisher',
        namespace=namespace,
        executable='static_transform_publisher',
        output='screen',
        prefix=term_prefix,
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'camera'],
        on_exit=[OpaqueFunction(function=publish_shutdown)]
    )

    localization_health_monitor = Node(
        package='localization_health_monitor',
        name='localization_health_monitor',
        namespace=namespace,
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
        on_exit=[OpaqueFunction(function=publish_shutdown)]
    )

    # The System Controller manages all of the lifecycle nodes
    carma_system_controller = Node(
        package='system_controller',
        name='carma_system_controller',
        namespace=namespace,
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
        ],
        on_exit=[OpaqueFunction(function=publish_shutdown)]
    )

    # Create the launch description
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_show_img_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Add the actions to launch the carma subsystems
    ld.add_action(perception_container)
    ld.add_action(dead_reckoner)
    ld.add_action(ekf_localizer)
    ld.add_action(localization_health_monitor)
    ld.add_action(transform_publisher_node)
    ld.add_action(carma_system_controller)

    return ld
