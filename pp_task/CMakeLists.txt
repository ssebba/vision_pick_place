cmake_minimum_required(VERSION 3.20)
project(pp_task LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

option(PP_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor)
  if(PP_WARNINGS_AS_ERRORS)
    add_compile_options(-Werror)
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(perception_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

generate_parameter_library(task_manager_parameters src/task_manager_parameters.yaml)

# Shared state of the tree: context, helpers, grasp attacher.
add_library(pp_task_context SHARED src/task_context.cpp src/grasp_attacher.cpp)
target_include_directories(pp_task_context PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(pp_task_context PUBLIC
  moveit_ros_planning_interface::moveit_move_group_interface
  perception_core::perception_core
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${geometry_msgs_TARGETS}
  ${moveit_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${tf2_msgs_TARGETS}
  ${vision_msgs_TARGETS}
)

# Behavior tree nodes, as a BT.CPP plugin (BT_REGISTER_NODES).
add_library(pp_bt_nodes SHARED src/bt_nodes.cpp)
target_compile_definitions(pp_bt_nodes PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(pp_bt_nodes PUBLIC
  pp_task_context
  behaviortree_cpp::behaviortree_cpp
  moveit_core::moveit_robot_state
  moveit_core::moveit_robot_trajectory
  moveit_core::moveit_trajectory_processing
)

# Lifecycle component running the tree.
add_library(${PROJECT_NAME} SHARED src/task_manager.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC
  pp_task_context
  task_manager_parameters
  ament_index_cpp::ament_index_cpp
  behaviortree_cpp::behaviortree_cpp
  rclcpp_action::rclcpp_action
  rclcpp_components::component
  rclcpp_lifecycle::rclcpp_lifecycle
  ${control_msgs_TARGETS}
)
rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "pp_task::TaskManager" EXECUTABLE task_manager)

install(TARGETS pp_task_context pp_bt_nodes ${PROJECT_NAME} task_manager_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_task_context test/test_task_context.cpp)
  target_link_libraries(test_task_context pp_task_context)
  ament_add_gtest(test_behavior_tree test/test_behavior_tree.cpp)
  target_link_libraries(test_behavior_tree pp_task_context behaviortree_cpp::behaviortree_cpp)
  target_compile_definitions(test_behavior_tree PRIVATE
    BT_PLUGIN_PATH="$<TARGET_FILE:pp_bt_nodes>"
    TREE_FILE="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees/pick_place.xml")
  add_dependencies(test_behavior_tree pp_bt_nodes)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_index_cpp behaviortree_cpp control_msgs generate_parameter_library geometry_msgs moveit_core moveit_msgs
  moveit_ros_planning_interface perception_core rclcpp rclcpp_action rclcpp_components rclcpp_lifecycle std_msgs
  tf2_msgs vision_msgs
)
ament_package()
