cmake_minimum_required(VERSION 3.8)
project(alidade_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/measure_odom.py DESTINATION lib/${PROJECT_NAME})

ament_package()
