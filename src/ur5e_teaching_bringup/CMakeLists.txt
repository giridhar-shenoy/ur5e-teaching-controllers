cmake_minimum_required(VERSION 3.16)
project(ur5e_teaching_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config urdf worlds DESTINATION share/${PROJECT_NAME})

ament_package()
