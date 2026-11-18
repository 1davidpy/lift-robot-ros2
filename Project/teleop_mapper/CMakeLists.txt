cmake_minimum_required(VERSION 3.16)
project(luxi_teleop_mapper)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(luxi_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(luxi_core REQUIRED)

add_executable(teleop_mapper src/teleop_mapper.cpp)
ament_target_dependencies(teleop_mapper rclcpp luxi_interfaces luxi_core)
install(TARGETS teleop_mapper DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME mapper_input_and_response
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mapper.py
      ${LUXI_ROOT} --binary $<TARGET_FILE:teleop_mapper>
      --config ${CMAKE_CURRENT_SOURCE_DIR}/../robot_bringup/config/arms-teleop.json
      --cutoff 4.0 --expect-left-open -0.5 --max-step90-ms 140
      --output-dir ${CMAKE_CURRENT_BINARY_DIR}/mapper-test-output)
  set_tests_properties(mapper_input_and_response PROPERTIES TIMEOUT 30)
  add_test(NAME mapper_wrist_roll
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_wrist_roll.py
      --binary $<TARGET_FILE:teleop_mapper>
      --config ${CMAKE_CURRENT_SOURCE_DIR}/../robot_bringup/config/arms-teleop.json)
  set_tests_properties(mapper_wrist_roll PROPERTIES TIMEOUT 30)
endif()
ament_package()
