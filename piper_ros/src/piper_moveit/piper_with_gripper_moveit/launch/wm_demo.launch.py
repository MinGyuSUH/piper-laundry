from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare argument
    declared_arguments = [
        DeclareLaunchArgument("real", default_value="false", description="real robot OR sim")
    ]
    real = LaunchConfiguration("real")


    moveit_config = (
        MoveItConfigsBuilder("piper", package_name="piper_with_gripper_moveit")
        .robot_description(mappings={"real": real})
        .robot_description_kinematics()
        .to_moveit_configs()
    )

    demo_launch = generate_demo_launch(moveit_config)

    planning_scene_node = Node(
        package="piper_with_gripper_moveit",
        executable="planning_scene_node",
        name="planning_scene_node",
        output="screen"
    )
    
    # 추가 노드: pose_goal_node
    pose_goal_node = Node(
        package="piper_with_gripper_moveit",
        executable="pose_goal_node",
        name="pose_goal_node",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )


    # Joint Goal 액션 서버 노드 (arm과 gripper 제어용)
    joint_goal_node = Node(
        package="piper_with_gripper_moveit",
        executable="joint_goal_node", # CMakeLists.txt에 정의한 이름
        name="joint_goal_node",       # 노드 이름 (자유롭게 지정 가능)
        output="screen",
        # 이 노드도 MoveIt 설정을 알아야 하므로 파라미터를 전달합니다.
        parameters=[moveit_config.to_dict()],
    )


    ld = LaunchDescription()

    for arg in declared_arguments:
        ld.add_action(arg)
    
    for action in demo_launch.entities:
        ld.add_action(action)

    ld.add_action(planning_scene_node)
    ld.add_action(pose_goal_node)
    ld.add_action(joint_goal_node) 

    return ld