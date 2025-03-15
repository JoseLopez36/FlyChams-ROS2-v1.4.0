from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction
import tempfile
import os

def generate_launch_description():
    # Declare launch arguments
    agent_id_arg = DeclareLaunchArgument(
        'agent_id',
        default_value='',
        description='Agent ID for namespace'
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
    pluginlists_path = PathJoinSubstitution([
        FindPackageShare('flychams_bringup'),
        'config',
        'mavros',
        'pluginlists.yaml'
    ])
    
    # Create a temporary directory for our modified config
    temp_dir = tempfile.mkdtemp()
    
    # Create an OpaqueFunction to handle the config file modification at runtime
    def launch_setup(context):
        agent_id = context.launch_configurations['agent_id']
        
        # Resolve the config path
        config_path_resolved = os.path.join(
            get_package_share_directory('flychams_bringup'),
            'config',
            'mavros',
            'config.yaml'
        )
        
        # Create a config file with the agent_id substituted
        temp_config_path = os.path.join(temp_dir, 'config.yaml')
        
        # Read, modify and write the config file
        with open(config_path_resolved, 'r') as original_file:
            content = original_file.read()
            modified_content = content.replace('AGENTID', agent_id)
        
        with open(temp_config_path, 'w') as temp_file:
            temp_file.write(modified_content)
        
        # Return the node with the modified config
        return [
            Node(
                package='mavros',
                executable='mavros_node',
                output='screen',
                namespace=['mavros/', LaunchConfiguration('agent_id')],
                parameters=[
                    pluginlists_path,
                    temp_config_path,
                    {    
                        'fcu_url': LaunchConfiguration('fcu_url'),
                        'gcs_url': LaunchConfiguration('gcs_url'),
                        'tgt_system': LaunchConfiguration('tgt_system'),
                        'tgt_component': LaunchConfiguration('tgt_component'),
                        'fcu_protocol': LaunchConfiguration('fcu_protocol'),
                        'respawn_mavros': LaunchConfiguration('respawn_mavros')
                    }
                ],
            )
        ]

    return LaunchDescription([
        agent_id_arg,
        fcu_url_arg,
        gcs_url_arg,
        tgt_system_arg,
        tgt_component_arg,
        fcu_protocol_arg,
        respawn_mavros_arg,
        
        # Call the opaque function after all arguments are declared
        OpaqueFunction(function=launch_setup),
    ])