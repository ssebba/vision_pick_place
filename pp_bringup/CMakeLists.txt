cmake_minimum_required(VERSION 3.20)
project(pp_bringup NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/run_trials.sh DESTINATION lib/${PROJECT_NAME})

ament_package()
