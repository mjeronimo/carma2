
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    bringup_dir = get_package_share_directory('carma_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_term = LaunchConfiguration('use_term')

    #term_prefix = 'xterm -geometry 150x40 -hold -e'
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
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    carma_nodes = [
        'carma_delphi_srr2_driver', 
        'carma_velodyne_lidar_driver',
        'dead_reckoner', 
        'ekf_localizer',
        'camera_driver',
        'camera_driver_client'
        ]

    # Nodes in the Perception Subsystem
    carma_delphi_srr2_driver = Node(
        package='carma_delphi_srr2_driver',
        executable='carma_delphi_srr2_driver',
        output='screen',
        prefix=term_prefix,
        respawn=True
        )
    carma_velodyne_lidar_driver = Node(
        package='carma_velodyne_lidar_driver',
        executable='carma_velodyne_lidar_driver',
        output='screen',
        prefix=term_prefix,
        )

    # Composable node container for the Perception Subsystem nodes
    container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='camera_driver',
                plugin='camera_driver::CameraDriver',
                name='camera_driver',
                extra_arguments=[{'use_intra_process_comms': True}]
                ),
            ComposableNode(
                package='camera_driver',
                plugin='camera_driver_client::CameraDriverClient',
                name='camera_driver_client',
                extra_arguments=[{'use_intra_process_comms': True}]
                ),
            ],
        output='screen',
        prefix=term_prefix,
        )

    # Localization Subsystem
    dead_reckoner = Node(
        package='dead_reckoner',
        executable='dead_reckoner',
        output='screen',
        prefix=term_prefix,
        )
        
    ekf_localizer = Node(
        package='ekf_localizer',
        executable='ekf_localizer',
        output='screen',
        prefix=term_prefix,
        )
    
    carma_system_controller = Node(
        package='system_controller',
        executable='system_controller',
        name='carma_system_controller',
        output='screen',
        prefix=term_prefix,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': carma_nodes}
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

    # Add the actions to launch the carma subsystems
    ld.add_action(carma_delphi_srr2_driver)
    ld.add_action(carma_velodyne_lidar_driver)
    ld.add_action(container)
    ld.add_action(dead_reckoner)
    ld.add_action(ekf_localizer)
    ld.add_action(carma_system_controller)

    return ld
