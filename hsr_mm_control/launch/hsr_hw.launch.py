import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    =========================================================================
    HSR MOBILE MANIPULATION DEMO: BUTTON PRESS
    =========================================================================
    
    BEFORE RUNNING THIS LAUNCH:
    
    1. Terminal 1 (Hardware/Motors):
       ros2 launch hsrb_bringup robot.launch.py
       (Wait for: "activated omni_base_controller")
       
    2. Terminal 2 (Robot State/TFs):
       ros2 launch hsrb_common_launch hsrb_common.launch.py
       
    3. Terminal 3 (This Launch):
       ros2 launch hsr_mm_control hsr_hw.launch.py
    =========================================================================
    """

    # Your Kinematic Setpoint Node
    # This node uses hardcoded XYZ coordinates to press the Ada button.
    kinematic_setpoint_node = Node(
        package='hsr_mm_control',
        executable='kinematic_setpoint_node',  # Ensure this matches your CMakeLists.txt
        name='kinematic_setpoint_node',
        output='screen',
        # Remappings ensure your node talks to the correct robot topics
        remappings=[
            ('/joint_states', '/whole_body/joint_states'),
            ('/odom', '/omni_base_controller/wheel_odom'),
            ('/cmd_vel', '/omni_base_controller/cmd_vel')
        ],
        parameters=[{
            'use_sim_time': False,  # Set to True only when running in Gazebo
        }]
    )

    return LaunchDescription([
        kinematic_setpoint_node
    ])