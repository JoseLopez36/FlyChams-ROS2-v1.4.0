from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import yaml
import launch.conditions
    
def generate_launch_description():
    # Define which nodes to launch
    nodes_to_launch = {
        # AirSim Wrapper nodes
        'airsim': True,
        # Bringup nodes
        'agent_registration': True,
        'target_registration': True,
        'cluster_registration': True,
        # Control nodes
        'agent_control': True,
        'head_control': True,
        # Perception nodes
        'clustering': True,
        # Coordination nodes
        'agent_positioning': True,
        'agent_assignment': True,
        'agent_tracking': True,
        # Dashboard nodes
        'gui': True,
        'visualization': True
    }

    # Define output and logging severity
    output_to_screen = {
        # AirSim Wrapper nodes
        'airsim': ['screen', 'error'],
        # Bringup nodes
        'agent_registration': ['screen', 'error'],
        'target_registration': ['screen', 'error'],
        'cluster_registration': ['screen', 'error'],
        # Control nodes
        'agent_control': ['screen', 'error'],
        'head_control': ['screen', 'error'],
        # Perception nodes
        'clustering': ['screen', 'error'],
        # Coordination nodes
        'agent_positioning': ['screen', 'error'],
        'agent_assignment': ['screen', 'error'],
        'agent_tracking': ['screen', 'error'],
        # Dashboard nodes
        'gui': ['screen', 'error'],
        'visualization': ['screen', 'error']
    }

    # Get package name
    package_name = 'flychams_bringup'

    # Get paths to config files
    # Core parameters
    system_path = PathJoinSubstitution([
        FindPackageShare(package_name),
        'config',
        'core',
        'system.yaml'
    ])
    topics_path = PathJoinSubstitution([
        FindPackageShare(package_name),
        'config',
        'core',
        'topics.yaml'
    ])
    frames_path = PathJoinSubstitution([
        FindPackageShare(package_name),
        'config',
        'core',
        'frames.yaml'
    ]) 
    params_path = PathJoinSubstitution([
        FindPackageShare(package_name),
        'config',
        'core',
        'params.yaml'
    ])
    
    # Package parameters
    bringup_path = PathJoinSubstitution([
        FindPackageShare(package_name),
        'config',
        'package',
        'bringup.yaml'
    ])
    control_path = PathJoinSubstitution([
        FindPackageShare(package_name),
        'config',
        'package',
        'control.yaml'
    ])
    perception_path = PathJoinSubstitution([
        FindPackageShare(package_name),
        'config',
        'package',
        'perception.yaml'
    ])
    coordination_path = PathJoinSubstitution([
        FindPackageShare(package_name),
        'config',
        'package',
        'coordination.yaml'
    ])
    dashboard_path = PathJoinSubstitution([
        FindPackageShare(package_name),
        'config',
        'package',
        'dashboard.yaml'
    ])

    # Set environment variable to control ROS logger output
    os.environ['RCUTILS_LOGGING_USE_STDOUT'] = '0' # Disable logging to stdout
    os.environ['RCUTILS_COLORIZED_OUTPUT'] = '1'   # Enable colored output

    # Generate launch description
    ld = []

    # ============= AIRSIM WRAPPER NODES =============
    # Conditionally add AirSim node
    if nodes_to_launch['airsim']:
        ld.append(
            Node(
                package='airsim_wrapper',
                executable='airsim_node',
                name='airsim_node',
                output=output_to_screen['airsim'][0],
                namespace='airsim',
                arguments=['--ros-args', '--log-level', output_to_screen['airsim'][1]],
                parameters=[{
                    'is_vulkan': True,
                    'update_airsim_state_every_n_sec': 0.050,
                    'update_sim_clock_every_n_sec': 0.001,
                    'update_airsim_status_every_n_sec': 0.100,
                    'publish_clock': True,
                    'host_ip': 'localhost',
                    'host_port': 41451,
                    'enable_api_control': True,
                    'enable_world_plot': False
                }]
            )
        )

    # ============= BRINGUP NODES =============
    # Conditionally add Agent Registration node
    if nodes_to_launch['agent_registration']:
        ld.append(
            Node(
                package='flychams_bringup',
                executable='agent_registration_node',
                name='agent_registration_node',
                output=output_to_screen['agent_registration'][0],
                namespace='flychams',
                arguments=['--ros-args', '--log-level', output_to_screen['agent_registration'][1]],
                parameters=[
                    system_path, 
                    topics_path, 
                    frames_path, 
                    params_path, 
                    bringup_path,
                    {'use_sim_time': True}
                ]
            )
        )

    # Conditionally add Target Registration node
    if nodes_to_launch['target_registration']:
        ld.append(
            Node(
                package='flychams_bringup',
                executable='target_registration_node',
                name='target_registration_node',
                output=output_to_screen['target_registration'][0],
                namespace='flychams',
                arguments=['--ros-args', '--log-level', output_to_screen['target_registration'][1]],
                parameters=[
                    system_path, 
                    topics_path, 
                    frames_path, 
                    params_path, 
                    bringup_path,
                    {'use_sim_time': True}
                ]
            )
        )

    # Conditionally add Cluster Registration node
    if nodes_to_launch['cluster_registration']:
        ld.append(
            Node(
                package='flychams_bringup',
                executable='cluster_registration_node',
                name='cluster_registration_node',
                output=output_to_screen['cluster_registration'][0],
                namespace='flychams',
                arguments=['--ros-args', '--log-level', output_to_screen['cluster_registration'][1]],
                parameters=[
                    system_path, 
                    topics_path, 
                    frames_path, 
                    params_path, 
                    bringup_path,
                    {'use_sim_time': True}
                ]
            )
        )

    # ============= CONTROL NODES =============
    # Conditionally add Agent Controller node
    if nodes_to_launch['agent_control']:
        ld.append(
            Node(
                package='flychams_control',
                executable='agent_control_node',
                name='agent_control_node',
                output=output_to_screen['agent_control'][0],
                namespace='flychams',
                arguments=['--ros-args', '--log-level', output_to_screen['agent_control'][1]],
                parameters=[
                    system_path, 
                    topics_path, 
                    frames_path, 
                    params_path, 
                    control_path,
                    {'use_sim_time': True}
                ]
            )
        )

    # Conditionally add Head Controller node
    if nodes_to_launch['head_control']:
        ld.append(
            Node(
                package='flychams_control',
                executable='head_control_node',
                name='head_control_node',
                output=output_to_screen['head_control'][0],
                namespace='flychams',
                arguments=['--ros-args', '--log-level', output_to_screen['head_control'][1]],
                parameters=[
                    system_path, 
                    topics_path, 
                    frames_path, 
                    params_path, 
                    control_path,
                    {'use_sim_time': True}
                ]
            )
        )

    # ============= PERCEPTION NODES =============
    # Conditionally add Clustering node
    if nodes_to_launch['clustering']:
        ld.append(
            Node(
                package='flychams_perception',
                executable='clustering_node',
                name='clustering_node',
                output=output_to_screen['clustering'][0],
                namespace='flychams',
                arguments=['--ros-args', '--log-level', output_to_screen['clustering'][1]],
                parameters=[
                    system_path, 
                    topics_path, 
                    frames_path, 
                    params_path, 
                    perception_path,
                    {'use_sim_time': True}
                ]
            )
        )

    # ============= COORDINATION NODES =============
    # Conditionally add Agent Positioning node
    if nodes_to_launch['agent_positioning']:
        ld.append(
            Node(
                package='flychams_coordination',
                executable='agent_positioning_node',
                name='agent_positioning_node',
                output=output_to_screen['agent_positioning'][0],
                namespace='flychams',
                arguments=['--ros-args', '--log-level', output_to_screen['agent_positioning'][1]],
                parameters=[
                    system_path, 
                    topics_path, 
                    frames_path, 
                    params_path, 
                    coordination_path,
                    {'use_sim_time': True}
                ]
            )
        )

    # Conditionally add Agent Assignment node
    if nodes_to_launch['agent_assignment']:
        ld.append(
            Node(
                package='flychams_coordination',
                executable='agent_assignment_node',
                name='agent_assignment_node',
                output=output_to_screen['agent_assignment'][0],
                namespace='flychams',
                arguments=['--ros-args', '--log-level', output_to_screen['agent_assignment'][1]],
                parameters=[
                    system_path, 
                    topics_path, 
                    frames_path, 
                    params_path, 
                    coordination_path,
                    {'use_sim_time': True}
                ]
            )
        )

    # Conditionally add Agent Tracking node
    if nodes_to_launch['agent_tracking']:
        ld.append(
            Node(
                package='flychams_coordination',
                executable='agent_tracking_node',
                name='agent_tracking_node',
                output=output_to_screen['agent_tracking'][0],
                namespace='flychams',
                arguments=['--ros-args', '--log-level', output_to_screen['agent_tracking'][1]],
                parameters=[
                    system_path, 
                    topics_path, 
                    frames_path, 
                    params_path, 
                    coordination_path,
                    {'use_sim_time': True}
                ]
            )
        )

    # ============= DASHBOARD NODES =============
    # Conditionally add GUI node
    if nodes_to_launch['gui']:
        ld.append(
            Node(
                package='flychams_dashboard',
                executable='gui_node',
                name='gui_node',
                output=output_to_screen['gui'][0],
                namespace='flychams',
                arguments=['--ros-args', '--log-level', output_to_screen['gui'][1]],
                parameters=[
                    system_path, 
                    topics_path, 
                    frames_path, 
                    params_path, 
                    dashboard_path,
                    {'use_sim_time': True}
                ]
            )
        )

    # Conditionally add Visualization node
    if nodes_to_launch['visualization']:
        ld.append(
            Node(
                package='flychams_dashboard',
                executable='visualization_node',
                name='visualization_node',
                output=output_to_screen['visualization'][0],
                namespace='flychams',
                arguments=['--ros-args', '--log-level', output_to_screen['visualization'][1]],
                parameters=[
                    system_path, 
                    topics_path, 
                    frames_path, 
                    params_path, 
                    dashboard_path,
                    {'use_sim_time': True}
                ]
            )
        )

    return LaunchDescription(ld)