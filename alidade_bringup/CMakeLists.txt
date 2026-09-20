cmake_minimum_required(VERSION 3.8)
project(alidade_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Module Python partage par les launch : rendu des gabarits et briques communes.
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/cmd_vel_stamper.py
  scripts/wait_for_scan.py
  scripts/system_monitor.py
  scripts/mode_manager.py
  scripts/aruco_bench.py
  scripts/camera_bench.py
  scripts/stone_detector.py
  scripts/docking_server.py
  scripts/gyro_odom.py
  scripts/odom_calib.py
  scripts/dock_campaign.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_params test/test_params.py)
  ament_add_pytest_test(test_telemetry test/test_telemetry.py)
  ament_add_pytest_test(test_modes test/test_modes.py)
  ament_add_pytest_test(test_stone test/test_stone.py)
  ament_add_pytest_test(test_docking test/test_docking.py)
  ament_add_pytest_test(test_odometry test/test_odometry.py)
endif()

ament_package()
