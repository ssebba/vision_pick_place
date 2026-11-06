cmake_minimum_required(VERSION 3.20)
project(arm_controllers LANGUAGES CXX)

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
find_package(arm_control_core REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)

generate_parameter_library(computed_torque_controller_parameters
  src/computed_torque_controller_parameters.yaml)
generate_parameter_library(cartesian_impedance_controller_parameters
  src/cartesian_impedance_controller_parameters.yaml)

add_library(${PROJECT_NAME} SHARED
  src/cartesian_impedance_controller.cpp
  src/computed_torque_controller.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  arm_control_core::arm_control_core
  cartesian_impedance_controller_parameters
  computed_torque_controller_parameters
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  ${control_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
)
pluginlib_export_plugin_description_file(controller_interface arm_controllers_plugins.xml)

install(TARGETS ${PROJECT_NAME}
    cartesian_impedance_controller_parameters computed_torque_controller_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ur_description REQUIRED)
  find_program(XACRO_EXECUTABLE xacro REQUIRED)
  set(_ur5e_urdf ${CMAKE_CURRENT_BINARY_DIR}/ur5e.urdf)
  add_custom_command(
    OUTPUT ${_ur5e_urdf}
    COMMAND ${XACRO_EXECUTABLE} ${ur_description_DIR}/../urdf/ur.urdf.xacro ur_type:=ur5e name:=ur
            -o ${_ur5e_urdf}
    COMMENT "Generating the UR5e URDF for the tests"
  )
  add_custom_target(controllers_ur5e_urdf ALL DEPENDS ${_ur5e_urdf})
  foreach(_test test_computed_torque_controller test_cartesian_impedance_controller)
    ament_add_gtest(${_test} test/${_test}.cpp TIMEOUT 120)
    target_link_libraries(${_test} ${PROJECT_NAME})
    target_compile_definitions(${_test} PRIVATE UR5E_URDF="${_ur5e_urdf}")
    add_dependencies(${_test} controllers_ur5e_urdf)
  endforeach()
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  arm_control_core control_msgs controller_interface generate_parameter_library geometry_msgs
  hardware_interface pluginlib rclcpp rclcpp_action rclcpp_lifecycle realtime_tools
)
ament_package()
