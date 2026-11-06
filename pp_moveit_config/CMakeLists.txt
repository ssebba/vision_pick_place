cmake_minimum_required(VERSION 3.20)
project(pp_moveit_config NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
