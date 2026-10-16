import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    
    # 1. Manus Core ROS 2 Clients
    manus_right_node = Node(
        package='manus_client',
        executable='manus_right',
        name='manus_right_client',
        output='screen'
    )

    manus_left_node = Node(
        package='manus_client',
        executable='manus_left',
        name='manus_left_client',
        output='screen'
    )

    # 2. FK & ZMQ Publishers (Manus Mocap Core)
    # Using ExecuteProcess to run these from the specific directory
    mocap_core_right = ExecuteProcess(
        cmd=['python3', 'manus_mocap_core.py'],
        cwd=os.path.expanduser('~/GeoRT/geort/mocap'),
        output='screen'
    )

    mocap_core_left = ExecuteProcess(
        cmd=['python3', 'manus_mocap_core_left.py'],
        cwd=os.path.expanduser('~/GeoRT/geort/mocap'),
        output='screen'
    )

    # 3. GeoRT Neural Network Streamer (Requires Conda)
    # We wrap the command to source conda and activate the env
    geort_streamer = ExecuteProcess(
        cmd=[
            'bash', '-c', 
            'source ~/anaconda3/etc/profile.d/conda.sh && '
            'conda activate geort && '
            'python3 geort/mocap/geort_streamer_dual.py'
        ],
        cwd=os.path.expanduser('~/GeoRT'),
        output='screen'
    )

    # 4. ROS 2 Hand Bridge
    # Replace 'your_hand_package' and 'hand_bridge_executable' with your actual names
    hand_bridge = ExecuteProcess(
        cmd=['python3', 'ros_joint_bridge_dual.py',
             '--right_addr', 'tcp://127.0.0.1:7777',
             '--left_addr', 'tcp://127.0.0.1:7778',
             '--right_topic', '/ra_/allegroHand_0/joint_cmd',
             '--left_topic', '/la_/allegroHand_1/joint_cmd',
             '--hz', '60',
             '--clamp'],
        cwd=os.path.expanduser('~/GeoRT/geort/mocap'),
        output='screen'
    )

    return LaunchDescription([
        # Start Manus Clients
        manus_right_node,
        manus_left_node,

        # Start Mocap Core scripts
        mocap_core_right,
        mocap_core_left,

        # Start Neural Network Streamer
        geort_streamer,

        # Start Final Bridge
        hand_bridge,
        
        LogInfo(msg="Finger mapping system initialized.")
    ])
