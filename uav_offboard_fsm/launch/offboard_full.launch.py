#!/usr/bin/env python3

"""
Combined launch file for the full UAV offboard stack.

Starts:
  - offboard_control_bridge  (traj_offboard)
  - online_traj_generator    (traj_offboard)
  - uav_offboard_fsm         (uav_offboard_fsm)
  - mock_px4_sim             (traj_offboard, conditional)
  - ros2 bag record          (conditional)

Originally two separate launch files:
  - traj_offboard/launch/offboard_traj.launch.py
  - uav_offboard_fsm/launch/uav_offboard_fsm.launch.py
"""

from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve_workspace_root(launch_file: Path) -> Path:
    """Walk up to find the ROS workspace root.

    A ROS workspace root has src/ containing packages, plus build/ and install/
    after a colcon build. Package-level src/ directories (containing C++/Python
    source files) should NOT match -- we skip those by requiring build/ or install/
    to also be present.
    """
    for parent in launch_file.parents:
        if (parent / 'src').is_dir() and (
            (parent / 'build').is_dir() or (parent / 'install').is_dir()
        ):
            return parent
    # Fallback for unbuilt workspaces: first parent whose src/ contains
    # subdirectories (packages), not just source files.
    for parent in launch_file.parents:
        src_dir = parent / 'src'
        if src_dir.is_dir():
            try:
                if any((src_dir / child).is_dir() for child in src_dir.iterdir()):
                    return parent
            except (OSError, PermissionError):
                continue
    return Path.cwd()


def generate_launch_description() -> LaunchDescription:
    launch_file = Path(__file__).resolve()
    workspace_dir = resolve_workspace_root(launch_file)
    bag_dir = workspace_dir / 'bag'
    bag_dir.mkdir(parents=True, exist_ok=True)
    bag_name = f"offboard_full_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    bag_output = str(bag_dir / bag_name)

    # ---- launch arguments ----
    use_mock_px4 = LaunchConfiguration('use_mock_px4')
    record_bag = LaunchConfiguration('record_bag')
    log_level = LaunchConfiguration('log_level')
    use_takeoff_on_ground = LaunchConfiguration('use_takeoff_on_ground')

    fsm_params_file = LaunchConfiguration('fsm_params_file')
    traj_params_file = LaunchConfiguration('traj_params_file')

    default_fsm_params = (
        Path(get_package_share_directory('uav_offboard_fsm'))
        / 'config' / 'uav_offboard_fsm.yaml'
    )
    default_traj_params = (
        Path(get_package_share_directory('traj_offboard'))
        / 'config' / 'offboard_control.yaml'
    )

    ros_args = ['--ros-args', '--log-level', log_level]

    # ---- rosbag topics (union of both original lists) ----
    topics_to_record = [
        '/online_traj_generator/ruckig_state',
        '/online_traj_generator/ruckig_command',
        '/online_traj_generator/ruckig_targ',
        '/uav_offboard_fsm/status',
        '/uav_offboard_fsm/status_text',
        '/uav_offboard_fsm/offboard_state',
        '/uav_offboard_fsm/control_command',
        '/fmu/out/vehicle_local_position',
        '/fmu/out/vehicle_attitude',
        '/fmu/out/vehicle_status',
        '/fmu/out/home_position',
        '/fmu/out/vehicle_status_v1',
        '/fmu/in/trajectory_setpoint',
        '/fmu/in/offboard_control_mode',
        '/fmu/in/vehicle_command',
        '/main_task_fsm/task_states',
        '/whole_body_planner/uav_setpoint',
    ]

    # ---- nodes ----
    offboard_bridge = Node(
        package='traj_offboard',
        executable='offboard_control_bridge_node',
        name='offboard_control_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[traj_params_file,
                    {'use_takeoff_on_ground': use_takeoff_on_ground}],
        arguments=ros_args,
    )

    traj_generator = Node(
        package='traj_offboard',
        executable='online_traj_node',
        name='online_traj_generator',
        output='screen',
        emulate_tty=True,
        arguments=ros_args,
    )

    fsm = Node(
        package='uav_offboard_fsm',
        executable='uav_offboard_fsm_node',
        name='uav_offboard_fsm',
        output='screen',
        emulate_tty=True,
        parameters=[fsm_params_file],
        arguments=ros_args,
    )

    mock_px4 = Node(
        package='traj_offboard',
        executable='mock_px4_sim_node',
        name='mock_px4_sim',
        output='screen',
        emulate_tty=True,
        arguments=ros_args,
        condition=IfCondition(use_mock_px4),
    )

    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '--storage', 'sqlite3',
            '--output', bag_output,
            *topics_to_record,
        ],
        output='screen',
        condition=IfCondition(record_bag),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_mock_px4',
            default_value='false',
            description='true to launch the lightweight PX4 topic simulator'),
        DeclareLaunchArgument(
            'record_bag',
            default_value='true',
            description='true to record trajectory and FSM topics'),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='ROS log level for all nodes'),
        DeclareLaunchArgument(
            'use_takeoff_on_ground',
            default_value='true',
            description='true for bridge-controlled ground takeoff; '
                        'false for pilot takeoff then manual OFFBOARD handoff'),
        DeclareLaunchArgument(
            'fsm_params_file',
            default_value=str(default_fsm_params),
            description='YAML parameter file for uav_offboard_fsm_node'),
        DeclareLaunchArgument(
            'traj_params_file',
            default_value=str(default_traj_params),
            description='YAML parameter file for offboard_control_bridge'),
        LogInfo(msg=['offboard_full: fsm_params_file=', fsm_params_file]),
        LogInfo(msg=['offboard_full: traj_params_file=', traj_params_file]),
        LogInfo(msg=['offboard_full: rosbag output=', bag_output],
                condition=IfCondition(record_bag)),
        offboard_bridge,
        traj_generator,
        fsm,
        mock_px4,
        rosbag_record,
    ])
