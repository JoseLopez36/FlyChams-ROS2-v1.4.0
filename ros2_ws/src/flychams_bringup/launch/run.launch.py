from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import launch.conditions
import os
    
def generate_launch_description():
    # Define logging level for each node
    log_level = {
        # Control nodes
        'drone_control': 'info',
        'head_control': 'info',
        # Perception nodes
        'target_clustering': 'info',
        'cluster_analysis': 'info',
        # Coordination nodes
        'agent_assignment': 'info',
        'agent_analysis': 'info',
        'agent_positioning': 'info',
        'agent_tracking': 'info',
        # Dashboard nodes
        'gui': 'info',
        'visualization': 'info',
        # Targets nodes
        'target_state': 'info',
        'target_control': 'info'
    }

    # Define launch arguments for each node
    launch_drone_control = LaunchConfiguration('drone_control')
    launch_head_control = LaunchConfiguration('head_control')
    launch_target_clustering = LaunchConfiguration('target_clustering')
    launch_cluster_analysis = LaunchConfiguration('cluster_analysis')
    launch_agent_assignment = LaunchConfiguration('agent_assignment')
    launch_agent_analysis = LaunchConfiguration('agent_analysis')
    launch_agent_positioning = LaunchConfiguration('agent_positioning')
    launch_agent_tracking = LaunchConfiguration('agent_tracking')
    launch_gui = LaunchConfiguration('gui')
    launch_visualization = LaunchConfiguration('visualization')
    launch_target_state = LaunchConfiguration('target_state')
    launch_target_control = LaunchConfiguration('target_control')

    # Define log level arguments for each node
    log_level_drone_control = LaunchConfiguration('log_drone_control')
    log_level_head_control = LaunchConfiguration('log_head_control')
    log_level_target_clustering = LaunchConfiguration('log_target_clustering')
    log_level_cluster_analysis = LaunchConfiguration('log_cluster_analysis')
    log_level_agent_assignment = LaunchConfiguration('log_agent_assignment')
    log_level_agent_analysis = LaunchConfiguration('log_agent_analysis')
    log_level_agent_positioning = LaunchConfiguration('log_agent_positioning')
    log_level_agent_tracking = LaunchConfiguration('log_agent_tracking')
    log_level_gui = LaunchConfiguration('log_gui')
    log_level_visualization = LaunchConfiguration('log_visualization')
    log_level_target_state = LaunchConfiguration('log_target_state')
    log_level_target_control = LaunchConfiguration('log_target_control')

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
    os.environ['RCUTILS_LOGGING_USE_STDOUT'] = '0' # Disable logging to stdout
    os.environ['RCUTILS_COLORIZED_OUTPUT'] = '1'   # Enable colored output

    # Generate launch description
    ld = []

    # Declare launch arguments with default values
    ld.append(DeclareLaunchArgument(
        'drone_control',
        default_value='True',
        description='Flag to enable/disable the Drone Control node'))
    
    ld.append(DeclareLaunchArgument(
        'head_control',
        default_value='True',
        description='Flag to enable/disable the Head Control node'))
    
    ld.append(DeclareLaunchArgument(
        'clustering',
        default_value='True',
        description='Flag to enable/disable the Clustering node'))
    
    ld.append(DeclareLaunchArgument(
        'agent_assignment',
        default_value='True',
        description='Flag to enable/disable the Agent Assignment node'))
    
    ld.append(DeclareLaunchArgument(
        'agent_analysis',
        default_value='True',
        description='Flag to enable/disable the Agent Analysis node'))
    
    ld.append(DeclareLaunchArgument(
        'agent_positioning',
        default_value='True',
        description='Flag to enable/disable the Agent Positioning node'))
    
    ld.append(DeclareLaunchArgument(
        'agent_tracking',
        default_value='True',
        description='Flag to enable/disable the Agent Tracking node'))
    
    ld.append(DeclareLaunchArgument(
        'gui',
        default_value='True',
        description='Flag to enable/disable the GUI node'))
    
    ld.append(DeclareLaunchArgument(
        'visualization',
        default_value='True',
        description='Flag to enable/disable the Visualization node'))
    
    ld.append(DeclareLaunchArgument(
        'target_state',
        default_value='True',
        description='Flag to enable/disable the Target State node'))
    
    ld.append(DeclareLaunchArgument(
        'target_control',
        default_value='True',
        description='Flag to enable/disable the Target Control node'))
    
    # Declare log level arguments with default values
    ld.append(DeclareLaunchArgument(
        'log_drone_control',
        default_value='info',
        description='Log level for the Drone Control node'))
    
    ld.append(DeclareLaunchArgument(
        'log_head_control',
        default_value='info',
        description='Log level for the Head Control node'))
    
    ld.append(DeclareLaunchArgument(
        'log_clustering',
        default_value='info',
        description='Log level for the Clustering node'))
    
    ld.append(DeclareLaunchArgument(
        'log_agent_assignment',
        default_value='info',
        description='Log level for the Agent Assignment node'))

    ld.append(DeclareLaunchArgument(
        'log_agent_analysis',
        default_value='info',
        description='Log level for the Agent Analysis node'))
    
    ld.append(DeclareLaunchArgument(
        'log_agent_positioning',
        default_value='info',
        description='Log level for the Agent Positioning node'))
    
    ld.append(DeclareLaunchArgument(
        'log_agent_tracking',
        default_value='info',
        description='Log level for the Agent Tracking node'))
    
    ld.append(DeclareLaunchArgument(
        'log_gui',
        default_value='info',
        description='Log level for the GUI node'))
    
    ld.append(DeclareLaunchArgument(
        'log_visualization',
        default_value='info',
        description='Log level for the Visualization node'))
    
    ld.append(DeclareLaunchArgument(
        'log_target_state',
        default_value='info',
        description='Log level for the Target State node'))
    
    ld.append(DeclareLaunchArgument(
        'log_target_control',
        default_value='info',
        description='Log level for the Target Control node'))

    # ============= CONTROL NODES =============
    # Conditionally add Drone Controller node
    ld.append(
        Node(
            package='flychams_control',
            executable='drone_control_node',
            name='drone_control_node',
            output='screen',
            namespace='flychams',
            condition=IfCondition(launch_drone_control),
            arguments=['--ros-args', '--log-level', log_level_drone_control],
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
    ld.append(
        Node(
            package='flychams_control',
            executable='head_control_node',
            name='head_control_node',
            output='screen',
            namespace='flychams',
            condition=IfCondition(launch_head_control),
            arguments=['--ros-args', '--log-level', log_level_head_control],
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
    ld.append(
        Node(
            package='flychams_perception',
            executable='target_clustering_node',
            name='target_clustering_node',
            output='screen',
            namespace='flychams',
            condition=IfCondition(launch_target_clustering),
            arguments=['--ros-args', '--log-level', log_level_target_clustering],
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
    ld.append(
        Node(
            package='flychams_perception',
            executable='cluster_analysis_node',
            name='cluster_analysis_node',
            output='screen',
            namespace='flychams',
            condition=IfCondition(launch_cluster_analysis),
            arguments=['--ros-args', '--log-level', log_level_cluster_analysis],
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
    ld.append(
        Node(
            package='flychams_coordination',
            executable='agent_assignment_node',
            name='agent_assignment_node',
            output='screen',
            namespace='flychams',
            condition=IfCondition(launch_agent_assignment),
            arguments=['--ros-args', '--log-level', log_level_agent_assignment],
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
    ld.append(
        Node(
            package='flychams_coordination',
            executable='agent_analysis_node',
            name='agent_analysis_node',
            output='screen',
            namespace='flychams',
            condition=IfCondition(launch_agent_analysis),
            arguments=['--ros-args', '--log-level', log_level_agent_analysis],
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
    ld.append(
        Node(
            package='flychams_coordination',
            executable='agent_positioning_node',
            name='agent_positioning_node',
            output='screen',
            namespace='flychams',
            condition=IfCondition(launch_agent_positioning),
            arguments=['--ros-args', '--log-level', log_level_agent_positioning],
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
    ld.append(
        Node(
            package='flychams_coordination',
            executable='agent_tracking_node',
            name='agent_tracking_node',
            output='screen',
            namespace='flychams',
            condition=IfCondition(launch_agent_tracking),
            arguments=['--ros-args', '--log-level', log_level_agent_tracking],
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
    # Conditionally add GUI node
    ld.append(
        Node(
            package='flychams_dashboard',
            executable='gui_node',
            name='gui_node',
            output='screen',
            namespace='flychams',
            condition=IfCondition(launch_gui),
            arguments=['--ros-args', '--log-level', log_level_gui],
            parameters=[
                system_path, 
                topics_path, 
                frames_path, 
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
            condition=IfCondition(launch_visualization),
            arguments=['--ros-args', '--log-level', log_level_visualization],
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
    ld.append(
        Node(
            package='flychams_targets',
            executable='target_state_node',
            name='target_state_node',
            output='screen',
            namespace='flychams',
            condition=IfCondition(launch_target_state),
            arguments=['--ros-args', '--log-level', log_level_target_state],
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
    ld.append(
        Node(
            package='flychams_targets',
            executable='target_control_node',
            name='target_control_node',
            output='screen',
            namespace='flychams',
            condition=IfCondition(launch_target_control),
            arguments=['--ros-args', '--log-level', log_level_target_control],
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