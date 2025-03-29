from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import launch.conditions
import os
import yaml
    
def generate_launch_description():
    # Get paths to config files
    # Core parameters
    system_path = PathJoinSubstitution([
        FindPackageShare('flychams_bringup'),
        'config',
        'core',
        'system.yaml'
    ])
    launch_path = PathJoinSubstitution([
        FindPackageShare('flychams_bringup'),
        'config',
        'core',
        'launch.yaml'
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
    
    # Package parameters
    control_path = PathJoinSubstitution([
        FindPackageShare('flychams_bringup'),
        'config',
        'package',
        'control.yaml'
    ])
    perception_path = PathJoinSubstitution([
        FindPackageShare('flychams_bringup'),
        'config',
        'package',
        'perception.yaml'
    ])
    coordination_path = PathJoinSubstitution([
        FindPackageShare('flychams_bringup'),
        'config',
        'package',
        'coordination.yaml'
    ])
    dashboard_path = PathJoinSubstitution([
        FindPackageShare('flychams_bringup'),
        'config',
        'package',
        'dashboard.yaml'
    ])
    targets_path = PathJoinSubstitution([
        FindPackageShare('flychams_bringup'),
        'config',
        'package',
        'targets.yaml'
    ])

    # Set environment variable to control ROS logger output
    os.environ['RCUTILS_LOGGING_USE_STDOUT'] = '1' # Enable logging to stdout
    os.environ['RCUTILS_COLORIZED_OUTPUT'] = '1'   # Enable colored output

    # Generate launch description
    ld = []

    # Load the nodes configuration YAML file
    # Convert from PathJoinSubstitution to path string and load the file
    launch_file_path = launch_path.perform(LaunchContext()).strip()
    with open(launch_file_path, 'r') as f:
        launch = yaml.safe_load(f)
    
    # Get the node activation settings from config
    # Set default values if not present in config
    nodes = {
        # Control nodes
        'drone_control': launch.get('drone_control', [True, 'info']),
        'head_control': launch.get('head_control', [True, 'info']),
        # Perception nodes
        'target_clustering': launch.get('target_clustering', [True, 'info']),
        'cluster_analysis': launch.get('cluster_analysis', [True, 'info']),
        # Coordination nodes
        'agent_assignment': launch.get('agent_assignment', [True, 'info']),
        'agent_analysis': launch.get('agent_analysis', [True, 'info']),
        'agent_positioning': launch.get('agent_positioning', [True, 'info']),
        'agent_tracking': launch.get('agent_tracking', [True, 'info']),
        # Targets nodes
        'target_state': launch.get('target_state', [True, 'info']),
        'target_control': launch.get('target_control', [True, 'info']),
        # Dashboard nodes
        'gui_manager': launch.get('gui_manager', [True, 'info']),
        'metrics_factory': launch.get('metrics_factory', [True, 'info']),
        'marker_factory': launch.get('marker_factory', [True, 'info'])
    }

    # ============= CONTROL NODES =============
    # Conditionally add Drone Controller node
    if nodes['drone_control'][0]:
        ld.append(
            Node(
                package='flychams_control',
                executable='drone_control_node',
                name='drone_control_node',
                output='screen',
                namespace='flychams',
                arguments=['--ros-args', '--log-level', nodes['drone_control'][1]],
                parameters=[
                    system_path, 
                    topics_path, 
                    frames_path, 
                    control_path,
                    {'use_sim_time': True}
                ]
            )
        )

    # Conditionally add Head Controller node
    if nodes['head_control'][0]:
        ld.append(
            Node(
                package='flychams_control',
                executable='head_control_node',
                name='head_control_node',
                output='screen',
                namespace='flychams',
                arguments=['--ros-args', '--log-level', nodes['head_control'][1]],
                parameters=[
                    system_path, 
                    topics_path, 
                    frames_path, 
                    control_path,
                    {'use_sim_time': True}
                ]
            )
        )

    # ============= PERCEPTION NODES =============
    # Conditionally add Target Clustering node
    if nodes['target_clustering'][0]:
        ld.append(
            Node(
                package='flychams_perception',
                executable='target_clustering_node',
                name='target_clustering_node',
                output='screen',
                namespace='flychams',
                arguments=['--ros-args', '--log-level', nodes['target_clustering'][1]],
                parameters=[
                    system_path, 
                    topics_path, 
                    frames_path, 
                    perception_path,
                    {'use_sim_time': True}
                ]
            )
        )

    # Conditionally add Cluster Analysis node
    if nodes['cluster_analysis'][0]:
        ld.append(
            Node(
                package='flychams_perception',
                executable='cluster_analysis_node',
                name='cluster_analysis_node',
                output='screen',
                namespace='flychams',
                arguments=['--ros-args', '--log-level', nodes['cluster_analysis'][1]],
                parameters=[
                    system_path, 
                    topics_path, 
                    frames_path, 
                    perception_path,
                    {'use_sim_time': True}
                ]
            )
        )

    # ============= COORDINATION NODES =============
    # Conditionally add Agent Assignment node
    if nodes['agent_assignment'][0]:
        ld.append(
            Node(
                package='flychams_coordination',
                executable='agent_assignment_node',
                name='agent_assignment_node',
                output='screen',
                namespace='flychams',
                arguments=['--ros-args', '--log-level', nodes['agent_assignment'][1]],
                parameters=[
                    system_path, 
                    topics_path, 
                    frames_path, 
                    coordination_path,
                    {'use_sim_time': True}
                ]
            )
        )

    # Conditionally add Agent Analysis node
    if nodes['agent_analysis'][0]:
        ld.append(
            Node(
                package='flychams_coordination',
                executable='agent_analysis_node',
                name='agent_analysis_node',
                output='screen',
                namespace='flychams',
                arguments=['--ros-args', '--log-level', nodes['agent_analysis'][1]],
                parameters=[
                    system_path, 
                    topics_path, 
                    frames_path, 
                    coordination_path,
                    {'use_sim_time': True}
                ]
            )
        )

    # Conditionally add Agent Positioning node
    if nodes['agent_positioning'][0]:
        ld.append(
            Node(
                package='flychams_coordination',
                executable='agent_positioning_node',
                name='agent_positioning_node',
                output='screen',
                namespace='flychams',
                arguments=['--ros-args', '--log-level', nodes['agent_positioning'][1]],
                parameters=[
                    system_path, 
                    topics_path, 
                    frames_path, 
                    coordination_path,
                    {'use_sim_time': True}
                ]
            )
        )

    # Conditionally add Agent Tracking node
    if nodes['agent_tracking'][0]:
        ld.append(
            Node(
                package='flychams_coordination',
                executable='agent_tracking_node',
                name='agent_tracking_node',
                output='screen',
                namespace='flychams',
                arguments=['--ros-args', '--log-level', nodes['agent_tracking'][1]],
                parameters=[
                    system_path, 
                    topics_path, 
                    frames_path, 
                    coordination_path,
                    {'use_sim_time': True}
                ]
            )
        )

    # ============= DASHBOARD NODES =============
    # Conditionally add GUI Manager node
    if nodes['gui_manager'][0]:
        ld.append(
            Node(
                package='flychams_dashboard',
                executable='gui_manager_node',
                name='gui_manager_node',
                output='screen',
                namespace='flychams',
                arguments=['--ros-args', '--log-level', nodes['gui_manager'][1]],
                parameters=[
                    system_path, 
                    topics_path, 
                    frames_path, 
                    dashboard_path,
                    {'use_sim_time': True}
                ]
            )
        )

    # Conditionally add Metrics Factory node
    if nodes['metrics_factory'][0]:
        ld.append(
            Node(
                package='flychams_dashboard',
                executable='metrics_factory_node',
                name='metrics_factory_node',
                output='screen',
                namespace='flychams',
                arguments=['--ros-args', '--log-level', nodes['metrics_factory'][1]],
                parameters=[
                    system_path, 
                    topics_path, 
                    frames_path, 
                    dashboard_path,
                    {'use_sim_time': True}
                ]
            )
        )

    # Conditionally add Marker Factory node
    if nodes['marker_factory'][0]:
        ld.append(
            Node(
                package='flychams_dashboard',
                executable='marker_factory_node',
                name='marker_factory_node',
                output='screen',
                namespace='flychams',
                arguments=['--ros-args', '--log-level', nodes['marker_factory'][1]],
                parameters=[
                    system_path, 
                    topics_path, 
                    frames_path, 
                    dashboard_path,
                    {'use_sim_time': True}
                ]
            )
        )

    # ============= TARGETS NODES =============
    # Conditionally add Target State node
    if nodes['target_state'][0]:
        ld.append(
            Node(
                package='flychams_targets',
                executable='target_state_node',
                name='target_state_node',
                output='screen',
                namespace='flychams',
                arguments=['--ros-args', '--log-level', nodes['target_state'][1]],
                parameters=[
                    system_path, 
                    topics_path, 
                    frames_path, 
                    targets_path,
                    {'use_sim_time': True}
                ]
            )
        )

    # Conditionally add Target Control node
    if nodes['target_control'][0]:
        ld.append(
            Node(
                package='flychams_targets',
                executable='target_control_node',
                name='target_control_node',
                output='screen',
                namespace='flychams',
                arguments=['--ros-args', '--log-level', nodes['target_control'][1]],
                parameters=[
                    system_path, 
                    topics_path, 
                    frames_path, 
                    targets_path,
                    {'use_sim_time': True}
                ]
            )
        )

    return LaunchDescription(ld)