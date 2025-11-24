from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml
import os


def generate_launch_description():
    # Declare Launch Args
    declared_arguments = [
        DeclareLaunchArgument("real", default_value="false", description="real robot OR sim"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("db", default_value="false"),
        DeclareLaunchArgument("debug", default_value="false")
    ]
    real = LaunchConfiguration("real")

    # Build moveit_config
    moveit_config = (
        MoveItConfigsBuilder("piper", package_name="piper_with_gripper_moveit")
        .robot_description(mappings={"real": real})
        .to_moveit_configs()
    )

    # Trajectory execution YAML
    trajectory_yaml = load_yaml('piper_with_gripper_moveit', 'config/trajectory_execution.yaml')

    launch_pkg_path = Path(moveit_config.package_path)

    ld = LaunchDescription(declared_arguments)

    # Include static_virtual_joint_tfs.launch.py (if exists)
    static_tf_path = launch_pkg_path / "launch/static_virtual_joint_tfs.launch.py"
    if static_tf_path.exists():
        ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(static_tf_path))))

    # Include robot_state_publisher
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_pkg_path / "launch/rsp.launch.py"))
    ))

    # Include move_group with patched parameters
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_pkg_path / "launch/move_group.launch.py")),
        launch_arguments=[('use_rviz', LaunchConfiguration('use_rviz'))]
    )
    # Patch move_group params by appending at runtime
    # You must edit move_group.launch.py to take parameters externally if necessary
    ld.add_action(move_group_launch)

    # Include RViz
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_pkg_path / "launch/moveit_rviz.launch.py")),
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    ))

    # Include warehouse_db
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_pkg_path / "launch/warehouse_db.launch.py")),
        condition=IfCondition(LaunchConfiguration("db")),
    ))

    # ros2_control_node
    ld.add_action(Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            str(launch_pkg_path / "config/ros2_controllers.yaml")
        ],
        output="screen"
    ))

    # Include spawn_controllers
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_pkg_path / "launch/spawn_controllers.launch.py"))
    ))

    # Include planning_scene_node
    ld.add_action(Node(
        package="piper_with_gripper_moveit",
        executable="planning_scene_node",
        name="planning_scene_node",
        output="screen"
    ))

    # 마지막으로 move_group 관련 trajectory_execution.yaml param을 patch
    # 이건 move_group.launch.py에서 `Node(..., parameters=[...])` 안에 trajectory_yaml도 포함되도록 수정해야 함

    return ld

def load_yaml(package_name, file_path):
    pkg_path = get_package_share_directory(package_name)
    full_path = os.path.join(pkg_path, file_path)
    with open(full_path, 'r') as f:
        return yaml.safe_load(f)