cmake_minimum_required(VERSION 3.20)
project(pp_perception LANGUAGES CXX)

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
find_package(generate_parameter_library REQUIRED)
find_package(perception_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

generate_parameter_library(object_detector_parameters src/object_detector_parameters.yaml)

add_library(${PROJECT_NAME} SHARED src/object_detector_node.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  object_detector_parameters
  perception_core::perception_core
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2::tf2
  tf2_eigen::tf2_eigen
  tf2_ros::tf2_ros
  ${sensor_msgs_TARGETS}
  ${vision_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "pp_perception::ObjectDetectorNode" EXECUTABLE object_detector)

install(TARGETS ${PROJECT_NAME} object_detector_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_object_detector_node test/test_object_detector_node.cpp TIMEOUT 60)
  target_link_libraries(test_object_detector_node ${PROJECT_NAME})

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_detector_launch.py TIMEOUT 120)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  generate_parameter_library perception_core rclcpp rclcpp_components
  rclcpp_lifecycle sensor_msgs tf2 tf2_eigen tf2_ros vision_msgs visualization_msgs
)
ament_package()
