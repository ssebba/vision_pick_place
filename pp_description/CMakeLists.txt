cmake_minimum_required(VERSION 3.20)
project(pp_description NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config urdf DESTINATION share/${PROJECT_NAME})

ament_package()
