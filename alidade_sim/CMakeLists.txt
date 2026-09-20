cmake_minimum_required(VERSION 3.8)
project(alidade_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY launch worlds models DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/measure_odom.py
  scripts/make_stone.py
  scripts/stones.py
  scripts/dock_sim_campaign.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_stone_model test/test_stone_model.py)
  ament_add_pytest_test(test_worlds test/test_worlds.py)
endif()

ament_package()
