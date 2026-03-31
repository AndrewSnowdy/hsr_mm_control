import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

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

def generate_launch_description():
    pkg_hsr_mm_control = get_package_share_directory('hsr_mm_control')

    # --- 1. Load URDF for the Ghost ---
    # The ghost needs the URDF to know where the arm links are relative to the base
    urdf_path = os.path.join(pkg_hsr_mm_control, 'urdf', 'hsrb.urdf')
    with open(urdf_path, 'r') as f:
        robot_description_content = f.read()

    # --- 2. Kinematic Setpoint Node (The Solver) ---
    kinematic_setpoint_node = Node(
        package='hsr_mm_control',
        executable='kinematic_setpoint_node',
        name='kinematic_setpoint_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # --- 3. Ghost Robot State Publisher ---
    # This solves the "No transform from ghost/arm_flex_link to map" error
    ghost_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ghost_robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_description': robot_description_content,
            'frame_prefix': 'ghost/'
        }],
        remappings=[
            ('/joint_states', '/ghost_joint_states'),
            ('/robot_description', '/ghost_robot_description')  # <-- ADD THIS
        ]
    )

    # --- 4. Mission Sequencer ---
    mission_sequencer_node = Node(
        package='hsr_mm_control',
        executable='mission_sequencer',
        name='mission_sequencer',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # Delay the sequencer by 5 seconds to let the IK solver warm up
    delayed_sequencer = TimerAction(
        period=5.0,
        actions=[mission_sequencer_node]
    )

    return LaunchDescription([
        kinematic_setpoint_node,
        ghost_rsp,
        delayed_sequencer
    ])