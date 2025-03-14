from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch.conditions
import os
    
def generate_launch_description():
    # Define logging level for each node
    log_level = {
        # Control nodes
        'agent_control': 'error',
        'head_control': 'error',
        # Perception nodes
        'clustering': 'error',
        # Coordination nodes
        'agent_positioning': 'error',
        'agent_assignment': 'error',
        'agent_tracking': 'error',
        # Dashboard nodes
        'gui': 'error',
        'visualization': 'error'
    }

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

    # Set environment variable to control ROS logger output
    os.environ['RCUTILS_LOGGING_USE_STDOUT'] = '0' # Disable logging to stdout
    os.environ['RCUTILS_COLORIZED_OUTPUT'] = '1'   # Enable colored output

    # Generate launch description
    ld = []

    # ============= CONTROL NODES =============
    # Conditionally add Agent Controller node
    ld.append(
        Node(
            package='flychams_control',
            executable='agent_control_node',
            name='agent_control_node',
            output='screen',
            namespace='flychams',
            arguments=['--ros-args', '--log-level', log_level['agent_control']],
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
    ld.append(
        Node(
            package='flychams_control',
            executable='head_control_node',
            name='head_control_node',
            output='screen',
            namespace='flychams',
            arguments=['--ros-args', '--log-level', log_level['head_control']],
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
    ld.append(
        Node(
            package='flychams_perception',
            executable='clustering_node',
            name='clustering_node',
            output='screen',
            namespace='flychams',
            arguments=['--ros-args', '--log-level', log_level['clustering']],
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
    ld.append(
        Node(
            package='flychams_coordination',
            executable='agent_positioning_node',
            name='agent_positioning_node',
            output='screen',
            namespace='flychams',
            arguments=['--ros-args', '--log-level', log_level['agent_positioning']],
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
    ld.append(
        Node(
            package='flychams_coordination',
            executable='agent_assignment_node',
            name='agent_assignment_node',
            output='screen',
            namespace='flychams',
            arguments=['--ros-args', '--log-level', log_level['agent_assignment']],
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
    ld.append(
        Node(
            package='flychams_coordination',
            executable='agent_tracking_node',
            name='agent_tracking_node',
            output='screen',
            namespace='flychams',
            arguments=['--ros-args', '--log-level', log_level['agent_tracking']],
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
    ld.append(
        Node(
            package='flychams_dashboard',
            executable='gui_node',
            name='gui_node',
            output='screen',
            namespace='flychams',
            arguments=['--ros-args', '--log-level', log_level['gui']],
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
    ld.append(
        Node(
            package='flychams_dashboard',
            executable='visualization_node',
            name='visualization_node',
            output='screen',
            namespace='flychams',
            arguments=['--ros-args', '--log-level', log_level['visualization']],
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