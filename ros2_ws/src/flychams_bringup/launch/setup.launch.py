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
                'update_airsim_status_every_n_sec': 0.100,
                'update_sim_clock_every_n_sec': 0.001,
                'enable_world_plot': False
                'host_ip': 'localhost',
                'host_port': 41451
            }]
        )
    )

    # ============= REGISTRATION NODES =============
    # Conditionally add Agent Registration node
    ld.append(
        Node(
            package='flychams_bringup',
            executable='agent_registration_node',
            name='agent_registration_node',
            output='screen',
            namespace='flychams',
            parameters=[
                system_path, 
                topics_path, 
                frames_path, 
                params_path, 
                {'use_sim_time': True}
            ]
        )
    )

    # Conditionally add Target Registration node
    ld.append(
        Node(
            package='flychams_bringup',
            executable='target_registration_node',
            name='target_registration_node',
            output='screen',
            namespace='flychams',
            parameters=[
                system_path, 
                topics_path, 
                frames_path, 
                params_path, 
                {'use_sim_time': True}
            ]
        )
    )

    # Conditionally add Cluster Registration node
    ld.append(
        Node(
            package='flychams_bringup',
            executable='cluster_registration_node',
            name='cluster_registration_node',
            output='screen',
            namespace='flychams',
            parameters=[
                system_path, 
                topics_path, 
                frames_path, 
                params_path, 
                {'use_sim_time': True}
            ]
        )
    )

    return LaunchDescription(ld)