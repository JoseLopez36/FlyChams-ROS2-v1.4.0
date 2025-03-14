import launch
import launch.actions
import datetime
import os

def generate_launch_description():
    # Define constant IDs to be recorded
    AGENT_IDS = ['AGENT04']
    TARGET_IDS = ['GROUP02_TARGET00', 'GROUP02_TARGET01', 'GROUP02_TARGET02', 'GROUP02_TARGET03', 'GROUP02_TARGET04',
                  'GROUP02_TARGET05', 'GROUP02_TARGET06', 'GROUP02_TARGET07', 'GROUP02_TARGET08', 'GROUP02_TARGET09', 
                  'GROUP02_TARGET10', 'GROUP02_TARGET11', 'GROUP02_TARGET12', 'GROUP02_TARGET13', 'GROUP02_TARGET14', 
                  'GROUP02_TARGET15']
    CLUSTER_IDS = ['CLUSTER00', 'CLUSTER01', 'CLUSTER02', 'CLUSTER03']

    # Build list of topics to record
    topics = []

    # Loop over agent IDs to add agent metrics topics
    for agent in AGENT_IDS:
        topics.append(f'/flychams/{agent}/metrics')

    # Loop over target IDs to add target metrics topics
    for target in TARGET_IDS:
        topics.append(f'/flychams/{target}/metrics')

    # Loop over cluster IDs to add cluster metrics topics
    for cluster in CLUSTER_IDS:
        topics.append(f'/flychams/{cluster}/metrics')

    # Append the global metrics topic
    topics.append('/flychams/global_metrics')

    # Generate a unique output directory name using an index
    # Check existing directories to find the next available index
    base_dir = '/home/testuser/FlyingChameleons/experiments/rosbags'
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