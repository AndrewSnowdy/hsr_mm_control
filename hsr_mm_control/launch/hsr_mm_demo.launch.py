import os
from ament_index_python.packages import get_package_share_directory
from hsrb_launch_utils.hsrb_launch_utils import declare_launch_arguments
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    declared_arguments = declare_launch_arguments()

    # --- Launch Arguments ---
    show_ghost_arg = DeclareLaunchArgument(
        'show_ghost', default_value='true',
        description='Run the ghost IK node and state publisher'
    )
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='full_sim',
        description='Simulation mode: full_sim | behavior'
    )

    condition    = LaunchConfiguration('fast_physics')
    show_ghost   = LaunchConfiguration('show_ghost')
    mode         = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    pkg_hsr_mm_control = get_package_share_directory('hsr_mm_control')

    # --- URDF ---
    urdf_path = os.path.join(pkg_hsr_mm_control, 'urdf', 'hsrb.urdf')
    with open(urdf_path, 'r') as f:
        robot_description_content = f.read()

    # --- Mode conditions ---
    is_full_sim = PythonExpression(["'", mode, "' == 'full_sim'"])
    is_behavior = PythonExpression(["'", mode, "' == 'behavior'"])

    # --- Gazebo (same as original: UnlessCondition on fast_physics, just world swaps) ---
    hsrb_gazebo_common_path = os.path.join(
        get_package_share_directory('hsrb_gazebo_launch'),
        'launch/include/hsrb_gazebo_common.launch.py')

    launch_arg_info = {
        "map": os.path.join(get_package_share_directory('tmc_potential_maps'),
                     'maps/white_space/map.yaml'),
        "robot_pos_x": "0.0",
        "robot_pos_y": "0.0",
        "robot_pos_z": "0.0",
        "robot_rpy_Y": "0.0"
    }

    hsrb_gazebo_full_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hsrb_gazebo_common_path),
        launch_arguments={**launch_arg_info, **{"world_name": os.path.join(pkg_hsr_mm_control, 'worlds', 'full_sim.world')}}.items(),
        condition=IfCondition(PythonExpression(
            ["'", condition, "'.lower() != 'true' and '", mode, "' == 'full_sim'"]
        ))
    )

    hsrb_gazebo_behavior = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hsrb_gazebo_common_path),
        launch_arguments={**launch_arg_info, **{"world_name": os.path.join(pkg_hsr_mm_control, 'worlds', 'behavior.world')}}.items(),
        condition=IfCondition(PythonExpression(
            ["'", condition, "'.lower() != 'true' and '", mode, "' == 'behavior'"]
        ))
    )

    # --- Ghost Robot (unchanged from original) ---
    kinematic_setpoint_node = Node(
        package='hsr_mm_control',
        executable='kinematic_setpoint_node',
        name='kinematic_setpoint_node',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(show_ghost)
    )

    ghost_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ghost_robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content,
            'frame_prefix': 'ghost/'
        }],
        remappings=[('/joint_states', '/ghost_joint_states')],
        condition=IfCondition(show_ghost)
    )

    # --- Door Button Bridge (swaps based on mode) ---
    door_button_bridge_full_sim = ExecuteProcess(
        cmd=['python3', os.path.join(pkg_hsr_mm_control, 'worlds', 'full_sim_door_button_bridge.py')],
        output='screen',
        name='full_sim_door_button_bridge',
        condition=IfCondition(is_full_sim)
    )

    door_button_bridge_behavior = ExecuteProcess(
        cmd=['python3', os.path.join(pkg_hsr_mm_control, 'worlds', 'behavior_door_button_bridge.py')],
        output='screen',
        name='behavior_door_button_bridge',
        condition=IfCondition(is_behavior)
    )

    # --- Vision Nodes (swaps based on mode) ---
    sim_vision_node = Node(
        package='hsr_vision',
        executable='full_sim_vision_node',
        name='full_sim_vision_node',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(is_full_sim)
    )

    behavior_vision_node = Node(
        package='hsr_vision',
        executable='behavior_sim_vision_node',
        name='behavior_sim_vision_node',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(is_behavior)
    )

    # 1. The Joint Trajectory Controller
    joint_traj_node = Node(
        package='hsr_mm_control',
        executable='joint_trajectory_node',
        name='joint_trajectory_node',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(is_behavior)
    )

    # 2. The Mission Sequencer
    mission_sequencer_node = Node(
        package='hsr_mm_control',
        executable='mission_sequencer',
        name='mission_sequencer',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(is_behavior)
    )

    # 3. Wrap them both in a 10-second Delay
    delayed_nodes = TimerAction(
        period=8.0,
        actions=[mission_sequencer_node]
    )

    return LaunchDescription(declared_arguments + [
        show_ghost_arg,
        mode_arg,
        hsrb_gazebo_full_sim,
        hsrb_gazebo_behavior,
        kinematic_setpoint_node,
        ghost_rsp,
        door_button_bridge_full_sim,
        door_button_bridge_behavior,
        sim_vision_node,
        behavior_vision_node,
        joint_traj_node,
        delayed_nodes,
    ])