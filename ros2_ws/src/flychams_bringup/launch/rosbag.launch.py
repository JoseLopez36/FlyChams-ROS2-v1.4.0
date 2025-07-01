import launch
import launch.actions
import datetime
import os

def generate_launch_description():
    # Define constant IDs to be recorded
    AGENT_IDS = ['AGENT00', 'AGENT01', 'AGENT02', 'AGENT03', 'AGENT04', 'AGENT05', 'AGENT06', 'AGENT07']
    TARGET_IDS = ['TARGET00COUNT00', 'TARGET00COUNT01', 
                  'TARGET01COUNT00', 'TARGET01COUNT01', 'TARGET01COUNT02', 'TARGET01COUNT03',
                  'TARGET02COUNT00', 'TARGET02COUNT01', 'TARGET02COUNT02', 'TARGET02COUNT03', 'TARGET02COUNT04', 'TARGET02COUNT05', 'TARGET02COUNT06', 'TARGET02COUNT07',
                  'TARGET03COUNT00', 'TARGET03COUNT01', 'TARGET03COUNT02', 'TARGET03COUNT03', 'TARGET03COUNT04', 'TARGET03COUNT05', 'TARGET03COUNT06', 'TARGET03COUNT07', 'TARGET03COUNT08', 'TARGET03COUNT09', 'TARGET03COUNT10', 'TARGET03COUNT11', 'TARGET03COUNT12', 'TARGET03COUNT13', 'TARGET03COUNT14', 'TARGET03COUNT15']
    CLUSTER_IDS = ['CLUSTER00', 'CLUSTER01', 'CLUSTER02', 'CLUSTER03', 'CLUSTER04']

    # Build list of topics to record
    topics = []

    # Loop over agent IDs to add agent metrics topics
    for agent in AGENT_IDS:
        topics.append(f'/flychams/dashboard/{agent}/metrics')

    # Loop over target IDs to add target metrics topics
    for target in TARGET_IDS:
        topics.append(f'/flychams/dashboard/{target}/metrics')

    # Loop over cluster IDs to add cluster metrics topics
    for cluster in CLUSTER_IDS:
        topics.append(f'/flychams/dashboard/{cluster}/metrics')

    # Append the global metrics topic
    topics.append('/flychams/dashboard/global/metrics')

    # Generate a unique output directory name using an index
    # Check existing directories to find the next available index
    base_dir = '/home/testuser/FlyChams-ROS2/experiments/rosbags'
    index = 0
    
    # Find the highest existing index
    if os.path.exists(base_dir):
        existing_dirs = [d for d in os.listdir(base_dir) if d.startswith('rosbag_') and d[7:].isdigit()]
        if existing_dirs:
            highest_index = max([int(d[7:]) for d in existing_dirs])
            index = highest_index + 1
    
    output_dir = f'{base_dir}/rosbag_{index}'

    # Construct the command to run: 'ros2 bag record -o <output_dir> <topics>'
    rosbag_command = ['ros2', 'bag', 'record', '-o', output_dir] + topics

    # Create an ExecuteProcess action to run the rosbag record command
    rosbag_process = launch.actions.ExecuteProcess(
        cmd=rosbag_command,
        output='screen'
    )

    return launch.LaunchDescription([
        rosbag_process
    ]) 