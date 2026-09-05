cmake_minimum_required(VERSION 3.8)
project(alidade_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# Le protocole est une bibliotheque a part, sans dependance ROS : c'est ce
# qui permet de le tester au banc et de le reutiliser hors ros2_control.
add_library(pico_protocol STATIC src/pico_protocol.cpp src/serial_port.cpp)
target_include_directories(pico_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pico_protocol PUBLIC cxx_std_17)
set_target_properties(pico_protocol PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(${PROJECT_NAME} SHARED src/pico_system.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC pico_protocol)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  hardware_interface pluginlib rclcpp rclcpp_lifecycle diagnostic_msgs)

pluginlib_export_plugin_description_file(hardware_interface alidade_hardware.xml)

install(TARGETS ${PROJECT_NAME} pico_protocol
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS scripts/pico_bench.py scripts/faux_pico.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol pico_protocol)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle diagnostic_msgs)
ament_package()
