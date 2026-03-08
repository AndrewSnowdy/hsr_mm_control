from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Jetson Remote Inference
    # We chain the commands: source the workspace, THEN run the node.
    # Replace '~/hsr_ros2_ws' with the actual path on your Jetson if it differs.
    start_jetson_inference = ExecuteProcess(
        cmd=[
            'ssh', '-t', 'administrator@192.168.200.201', 
            'ros2 run hsr_vision inference_node'
        ],
        output='screen',
        shell=True
    )

    # 2. HSR Local Processing
    start_hsr_processing = Node(
        package='hsr_vision',
        executable='process_node',
        name='process_node',
        output='screen'
    )

    return LaunchDescription([
        LogInfo(msg="Launching Distributed Vision System with Remote Sourcing..."),
        start_jetson_inference,
        start_hsr_processing
    ])