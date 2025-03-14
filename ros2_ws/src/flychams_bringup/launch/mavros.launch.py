from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='mavros',
        description='Namespace for the mavros node'
    )
    
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/ttyACM0:57600',
        description='URL for FCU connection'
    )
    
    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='',
        description='URL for GCS connection'
    )
    
    tgt_system_arg = DeclareLaunchArgument(
        'tgt_system',
        default_value='1',
        description='Target system ID'
    )
    
    tgt_component_arg = DeclareLaunchArgument(
        'tgt_component',
        default_value='1',
        description='Target component ID'
    )
    
    fcu_protocol_arg = DeclareLaunchArgument(
        'fcu_protocol',
        default_value='v2.0',
        description='FCU protocol version'
    )
    
    respawn_mavros_arg = DeclareLaunchArgument(
        'respawn_mavros',
        default_value='false',
        description='Whether to respawn the mavros node'
    )
    
    # Config files
    px4_config_path = PathJoinSubstitution([
        FindPackageShare('flychams_bringup'),
        'config',
        'mavros',
        'config.yaml'
    ])
    
    px4_pluginlists_path = PathJoinSubstitution([
        FindPackageShare('flychams_bringup'),
        'config',
        'mavros',
        'pluginlists.yaml'
    ])

    return LaunchDescription([
        namespace_arg,
        fcu_url_arg,
        gcs_url_arg,
        tgt_system_arg,
        tgt_component_arg,
        fcu_protocol_arg,
        respawn_mavros_arg,
        
        Node(
            package='mavros',
            executable='mavros_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                px4_pluginlists_path,
                px4_config_path,
                {    
                    'fcu_url': LaunchConfiguration('fcu_url'),
                    'gcs_url': LaunchConfiguration('gcs_url'),
                    'tgt_system': LaunchConfiguration('tgt_system'),
                    'tgt_component': LaunchConfiguration('tgt_component'),
                    'fcu_protocol': LaunchConfiguration('fcu_protocol'),
                    'respawn_mavros': LaunchConfiguration('respawn_mavros'),
                    'namespace': LaunchConfiguration('namespace'),
                }
            ],
        ),
    ])