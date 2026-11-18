cmake_minimum_required(VERSION 3.16)
project(luxi_control_manager)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(luxi_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(luxi_core REQUIRED)

add_executable(control_manager src/control_manager.cpp)
ament_target_dependencies(control_manager rclcpp luxi_core luxi_interfaces std_msgs)
install(TARGETS control_manager DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  enable_testing()
  add_test(NAME manual_pose_handover
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_manual_pose_handover.py
      $<TARGET_FILE:control_manager>
      ${LUXI_ROOT}/build/packages/luxi_device_adapters/openarm_adapter
      ${CMAKE_CURRENT_SOURCE_DIR}/../robot_bringup/config/arms-kh.json)
  set_tests_properties(manual_pose_handover PROPERTIES TIMEOUT 90)
  add_test(NAME homing_tolerance
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_homing_tolerance.py
      $<TARGET_FILE:control_manager>
      ${CMAKE_CURRENT_SOURCE_DIR}/../robot_bringup/config/arms-kh.json)
  set_tests_properties(homing_tolerance PROPERTIES TIMEOUT 45)
  add_test(NAME network_recovery
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_network_recovery.py
      $<TARGET_FILE:control_manager>
      ${CMAKE_CURRENT_SOURCE_DIR}/../robot_bringup/config/arms-teleop.json)
  set_tests_properties(network_recovery PROPERTIES TIMEOUT 90)
  add_test(NAME tracking_gap
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_tracking_gap.py
      $<TARGET_FILE:control_manager>
      ${CMAKE_CURRENT_SOURCE_DIR}/../robot_bringup/config/arms-kh.json)
  set_tests_properties(tracking_gap PROPERTIES TIMEOUT 45)
  add_test(NAME phase_speeds
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_phase_speeds.py
      $<TARGET_FILE:control_manager>
      ${CMAKE_CURRENT_SOURCE_DIR}/../robot_bringup/config/arms-teleop.json)
  set_tests_properties(phase_speeds PROPERTIES TIMEOUT 30)
  add_test(NAME shared_parking
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_shared_parking.py
      $<TARGET_FILE:control_manager>
      ${CMAKE_CURRENT_SOURCE_DIR}/../robot_bringup/config/arms-teleop.json)
  set_tests_properties(shared_parking PROPERTIES TIMEOUT 120)
  add_test(NAME lift_position_isolation
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_lift_position_isolation.py
      $<TARGET_FILE:control_manager>
      ${CMAKE_CURRENT_SOURCE_DIR}/../robot_bringup/config/arms-teleop.json)
  set_tests_properties(lift_position_isolation PROPERTIES TIMEOUT 90)
endif()
ament_package()
