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
        'agent_control': 'info',
        # Perception nodes
        'clustering': 'info',
        # Coordination nodes
        'agent_positioning': 'info',
        'agent_assignment': 'info',
        'agent_tracking': 'info',
        # Dashboard nodes
        'gui': 'info',
        'visualization': 'info',
        # Targets nodes
        'target_control': 'info'
    }

    # Define launch arguments for each node
    launch_agent_control = LaunchConfiguration('launch_agent_control')
    launch_clustering = LaunchConfiguration('launch_clustering')
    launch_agent_positioning = LaunchConfiguration('launch_agent_positioning')
    launch_agent_assignment = LaunchConfiguration('launch_agent_assignment')
    launch_agent_tracking = LaunchConfiguration('launch_agent_tracking')
    launch_gui = LaunchConfiguration('launch_gui')
    launch_visualization = LaunchConfiguration('launch_visualization')
    launch_target_control = LaunchConfiguration('launch_target_control')

    # Define log level arguments for each node
    log_level_agent_control = LaunchConfiguration('log_level_agent_control')
    log_level_clustering = LaunchConfiguration('log_level_clustering')
    log_level_agent_positioning = LaunchConfiguration('log_level_agent_positioning')
    log_level_agent_assignment = LaunchConfiguration('log_level_agent_assignment')
    log_level_agent_tracking = LaunchConfiguration('log_level_agent_tracking')
    log_level_gui = LaunchConfiguration('log_level_gui')
    log_level_visualization = LaunchConfiguration('log_level_visualization')
    log_level_target_control = LaunchConfiguration('log_level_target_control')

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
        'launch_agent_control',
        default_value='False',
        description='Flag to enable/disable the Agent Control node'))
    
    ld.append(DeclareLaunchArgument(
        'launch_clustering',
        default_value='False',
        description='Flag to enable/disable the Clustering node'))
    
    ld.append(DeclareLaunchArgument(
        'launch_agent_positioning',
        default_value='False',
        description='Flag to enable/disable the Agent Positioning node'))
    
    ld.append(DeclareLaunchArgument(
        'launch_agent_assignment',
        default_value='False',
        description='Flag to enable/disable the Agent Assignment node'))
    
    ld.append(DeclareLaunchArgument(
        'launch_agent_tracking',
        default_value='False',
        description='Flag to enable/disable the Agent Tracking node'))
    
    ld.append(DeclareLaunchArgument(
        'launch_gui',
        default_value='False',
        description='Flag to enable/disable the GUI node'))
    
    ld.append(DeclareLaunchArgument(
        'launch_visualization',
        default_value='False',
        description='Flag to enable/disable the Visualization node'))
    
    ld.append(DeclareLaunchArgument(
        'launch_target_control',
        default_value='False',
        description='Flag to enable/disable the Target Control node'))
    
    # Declare log level arguments with default values
    ld.append(DeclareLaunchArgument(
        'log_level_agent_control',
        default_value='error',
        description='Log level for the Agent Control node'))
    
    ld.append(DeclareLaunchArgument(
        'log_level_clustering',
        default_value='error',
        description='Log level for the Clustering node'))
    
    ld.append(DeclareLaunchArgument(
        'log_level_agent_positioning',
        default_value='error',
        description='Log level for the Agent Positioning node'))
    
    ld.append(DeclareLaunchArgument(
        'log_level_agent_assignment',
        default_value='error',
        description='Log level for the Agent Assignment node'))
    
    ld.append(DeclareLaunchArgument(
        'log_level_agent_tracking',
        default_value='error',
        description='Log level for the Agent Tracking node'))
    
    ld.append(DeclareLaunchArgument(
        'log_level_gui',
        default_value='error',
        description='Log level for the GUI node'))
    
    ld.append(DeclareLaunchArgument(
        'log_level_visualization',
        default_value='error',
        description='Log level for the Visualization node'))
    
    ld.append(DeclareLaunchArgument(
        'log_level_target_control',
        default_value='error',
        description='Log level for the Target Control node'))

    # ============= CONTROL NODES =============
    # Conditionally add Agent Controller node
    ld.append(
        Node(
            package='flychams_control',
            executable='agent_control_node',
            name='agent_control_node',
            output='screen',
            namespace='flychams',
            condition=IfCondition(launch_agent_control),
            arguments=['--ros-args', '--log-level', log_level_agent_control],
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
            condition=IfCondition(launch_clustering),
            arguments=['--ros-args', '--log-level', log_level_clustering],
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
            condition=IfCondition(launch_agent_positioning),
            arguments=['--ros-args', '--log-level', log_level_agent_positioning],
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
            condition=IfCondition(launch_agent_assignment),
            arguments=['--ros-args', '--log-level', log_level_agent_assignment],
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
            condition=IfCondition(launch_agent_tracking),
            arguments=['--ros-args', '--log-level', log_level_agent_tracking],
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
            condition=IfCondition(launch_gui),
            arguments=['--ros-args', '--log-level', log_level_gui],
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
            condition=IfCondition(launch_visualization),
            arguments=['--ros-args', '--log-level', log_level_visualization],
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

    # ============= TARGETS NODES =============
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
                params_path, 
                targets_path,
                {'use_sim_time': True}
            ]
        )
    )

    return LaunchDescription(ld)