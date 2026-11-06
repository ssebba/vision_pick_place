cmake_minimum_required(VERSION 3.20)
project(arm_control_core LANGUAGES CXX)

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
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

# Pure C++ library; Pinocchio is a private dependency (pimpl in robot_model.cpp).
add_library(${PROJECT_NAME} SHARED
  src/control_laws.cpp
  src/joint_trajectory.cpp
  src/robot_model.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen PRIVATE pinocchio::pinocchio)

install(TARGETS ${PROJECT_NAME}
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
  # The tests run on the real UR5e model, generated from ur_description.
  set(_ur5e_urdf ${CMAKE_CURRENT_BINARY_DIR}/ur5e.urdf)
  add_custom_command(
    OUTPUT ${_ur5e_urdf}
    COMMAND ${XACRO_EXECUTABLE} ${ur_description_DIR}/../urdf/ur.urdf.xacro ur_type:=ur5e name:=ur
            -o ${_ur5e_urdf}
    COMMENT "Generating the UR5e URDF for the tests"
  )
  add_custom_target(ur5e_urdf ALL DEPENDS ${_ur5e_urdf})
  foreach(_test test_robot_model test_joint_trajectory test_control_laws)
    ament_add_gtest(${_test} test/${_test}.cpp TIMEOUT 120)
    target_link_libraries(${_test} ${PROJECT_NAME})
    target_compile_definitions(${_test} PRIVATE UR5E_URDF="${_ur5e_urdf}")
    add_dependencies(${_test} ur5e_urdf)
  endforeach()
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3 pinocchio)
ament_package()
