from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
    
def generate_launch_description():
    # Get paths to config files
    # Core parameters
    system_path = PathJoinSubstitution([
        FindPackageShare('flychams_bringup'),
        'config',
        'core',
        'system.yaml'
    ])
    topics_path = PathJoinSubstitution([
        FindPackageShare('flychams_bringup'),
        'config',
        'core',
        'topics.yaml'
    ])
    frames_path = PathJoinSubstitution([
        FindPackageShare('flychams_bringup'),
        'config',
        'core',
        'frames.yaml'
    ]) 
    params_path = PathJoinSubstitution([
        FindPackageShare('flychams_bringup'),
        'config',
        'core',
        'params.yaml'
    ])

    # Set environment variable to control ROS logger output
    os.environ['RCUTILS_LOGGING_USE_STDOUT'] = '0' # Disable logging to stdout
    os.environ['RCUTILS_COLORIZED_OUTPUT'] = '1'   # Enable colored output

    # Generate launch description
    ld = []

    # ============= AIRSIM WRAPPER NODES =============
    # Conditionally add AirSim node
    ld.append(
        Node(
            package='airsim_wrapper',
            executable='airsim_node',
            name='airsim_node',
            output='screen',
            namespace='airsim',
            parameters=[{
                'update_airsim_state_every_n_sec': 0.020,
                'update_sim_clock_every_n_sec': 0.001,
                'world_frame_id': 'world',
                'vehicle_local_frame_id': 'local',
                'vehicle_body_frame_id': 'body',
                'camera_body_frame_id': 'body',
                'camera_optical_frame_id': 'optical',
                'enable_world_plot': True,
                'host_ip': 'localhost',
                'host_port': 41451
            }]
        )
    )

    # ============= REGISTRATOR NODES =============
    # Conditionally add Registrator node
    ld.append(
        Node(
            package='flychams_bringup',
            executable='registrator_node',
            name='registrator_node',
            output='screen',
            namespace='flychams',
            parameters=[
                system_path, 
                topics_path, 
                frames_path, 
                params_path
            ]
        )
    )

    return LaunchDescription(ld)