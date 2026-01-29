import os
from ament_index_python.packages import get_package_share_directory
from hsrb_launch_utils.hsrb_launch_utils import declare_launch_arguments
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    declared_arguments = declare_launch_arguments()
    
    # NEW ARGUMENT: Toggle the ghost robot on/off
    show_ghost_arg = DeclareLaunchArgument(
        'show_ghost', default_value='true', 
        description='Run the ghost IK node and state publisher'
    )

    condition = LaunchConfiguration('fast_physics')
    show_ghost = LaunchConfiguration('show_ghost')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')


    pkg_hsr_mm_control = get_package_share_directory('hsr_mm_control')

    urdf_path = os.path.join(pkg_hsr_mm_control, 'urdf', 'hsrb.urdf')
    with open(urdf_path, 'r') as f:
        robot_description_content = f.read()


    # --- Gazebo Logic ---
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
    my_world_path = os.path.join(pkg_hsr_mm_control, 'worlds', 'hsr_mm.world')

    hsrb_gazebo_common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hsrb_gazebo_common_path),
        launch_arguments={**launch_arg_info, **{"world_name": my_world_path}}.items(),
        condition=UnlessCondition(condition))

    # --- NEW: Ghost Robot Infrastructure ---

    # 1. Your IK Node
    kinematic_setpoint_node = Node(
        package='hsr_mm_control',
        executable='kinematic_setpoint_node',
        name='kinematic_setpoint_node',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(show_ghost)
    )

    # 2. Ghost Robot State Publisher
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

    return LaunchDescription(declared_arguments + [
        show_ghost_arg,
        hsrb_gazebo_common,
        kinematic_setpoint_node,
        ghost_rsp
    ])