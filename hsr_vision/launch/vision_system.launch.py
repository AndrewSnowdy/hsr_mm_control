from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Jetson Remote Inference
    # We chain the commands: source the workspace, THEN run the node.
    # Replace '~/hsr_ros2_ws' with the actual path on your Jetson if it differs.
    # need to run ssh-copy-id neuroam@192.168.200.201 first to set up passwordless ssh
    start_jetson_inference = ExecuteProcess(
        cmd=[
            'ssh', '-tt', 'neuroam@192.168.200.201', 
            'export ROS_DOMAIN_ID=48 && source /opt/ros/humble/setup.bash && source ~/hsr_ros2_ws/install/setup.bash && ros2 run hsr_vision inference_node'
        ],
        output='screen'
        # shell=True REMOVED - ROS 2 will handle the execution directly
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