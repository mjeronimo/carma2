
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('carma_bringup')
   
    use_sim_time = LaunchConfiguration('use_sim_time')

    autostart = LaunchConfiguration('autostart')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

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


    
    driver_nodes = ['carma_delphi_srr2_driver', 'carma_velodyne_lidar_driver']
    world_model_nodes = ['roadway_objects', 'world_model_controller']

    # Subsystem 1: Drivers
    carma_delphi_srr2_driver = Node(
            package='carma_delphi_srr2_driver',
            executable='carma_delphi_srr2_driver',
            output='screen',
            namespace='perception_subsystem',
            )
    carma_velodyne_lidar_driver = Node(
            package='carma_velodyne_lidar_driver',
            executable='carma_velodyne_lidar_driver',
            output='screen',
            namespace='perception_subsystem',
            )
    drivers_lifecycle_manager = Node(
            package='ros2_lifecycle_manager',
            executable='lifecycle_manager',
            name='drivers_lifecycle_manager',
            output='screen',
            namespace='perception_subsystem',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': driver_nodes}])

    # Subsystem 2: World Models
    roadway_objects = Node(
            package='roadway_objects',
            executable='roadway_objects',
            output='screen',
            namespace='world_model_subsystem',
            )
    world_model_controller = Node(
            package='world_model_controller',
            executable='world_model_controller',
            output='screen',
            namespace='world_model_subsystem',
            )
    world_model_lifecycle_manager = Node(
            package='ros2_lifecycle_manager',
            executable='lifecycle_manager',
            name='world_model_lifecycle_manager',
            output='screen',
            namespace='world_model_subsystem',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': world_model_nodes}])

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

    # Add the actions to launch the perception subsystem
    ld.add_action(carma_delphi_srr2_driver)
    ld.add_action(carma_velodyne_lidar_driver)
    ld.add_action(drivers_lifecycle_manager)

    # Add the actions to launch the world model subsystem
    ld.add_action(roadway_objects)
    ld.add_action(world_model_controller)
    ld.add_action(world_model_lifecycle_manager)

    return ld
