cmake_minimum_required(VERSION 3.8)
project(alidade_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY launch worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/measure_odom.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_worlds test/test_worlds.py)
endif()

ament_package()
