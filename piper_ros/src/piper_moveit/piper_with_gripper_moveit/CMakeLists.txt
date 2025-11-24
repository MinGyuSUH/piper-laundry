cmake_minimum_required(VERSION 3.22)
project(piper_with_gripper_moveit)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(control_msgs REQUIRED)

find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)

find_package(rosidl_default_generators REQUIRED)

find_package(action_msgs REQUIRED)

find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PoseGoal.action"
  "action/TargetPose.action"
  DEPENDENCIES geometry_msgs builtin_interfaces std_msgs
)

add_executable(planning_scene_node src/planning_scene.cpp)
add_executable(joint_goal_node src/joint_goal.cpp)
# add_executable(joint_goal_cmd_node src/joint_goal_cmd.cpp)
add_executable(pose_goal_node src/pose_goal.cpp)

ament_target_dependencies(planning_scene_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

ament_target_dependencies(joint_goal_node
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  control_msgs
)

# ament_target_dependencies(joint_goal_cmd_node
#   rclcpp
#   moveit_ros_planning_interface
#   control_msgs
# )

ament_target_dependencies(pose_goal_node
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  geometry_msgs
  control_msgs
  tf2_geometry_msgs # CLIK
  Eigen3 # CLIK
)


rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
target_link_libraries(pose_goal_node
  ${cpp_typesupport_target}
)


ament_target_dependencies(planning_scene_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)


target_link_libraries(joint_goal_node pthread)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

install(TARGETS
  planning_scene_node
  joint_goal_node
  # joint_goal_cmd_node
  pose_goal_node
  DESTINATION lib/${PROJECT_NAME}
)

# Python 실행 파일 설치
install(
  PROGRAMS
    # src/cA.py
    src/Logic_inte_ba.py
    src/TEST.py
  DESTINATION lib/${PROJECT_NAME}
)
