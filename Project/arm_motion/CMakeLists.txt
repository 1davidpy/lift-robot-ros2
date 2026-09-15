cmake_minimum_required(VERSION 3.16)
project(luxi_arm_motion)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(luxi_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(luxi_core REQUIRED)

set(MUJOCO_ROOT "${LUXI_ROOT}/3part/external/mujoco")
find_library(MUJOCO_LIBRARY NAMES mujoco libmujoco.so.3.12.0 PATHS "${MUJOCO_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)
add_library(luxi_ik INTERFACE)
target_include_directories(luxi_ik INTERFACE include "${MUJOCO_ROOT}/include")
target_link_libraries(luxi_ik INTERFACE ${MUJOCO_LIBRARY})
add_executable(arm_motion src/arm_motion.cpp)
target_link_libraries(arm_motion luxi_ik)
ament_target_dependencies(arm_motion rclcpp luxi_interfaces luxi_core std_msgs)
set_target_properties(arm_motion PROPERTIES INSTALL_RPATH "$ORIGIN/../")
install(DIRECTORY "${MUJOCO_ROOT}/lib/" DESTINATION lib FILES_MATCHING PATTERN "libmujoco.so*")
install(TARGETS arm_motion DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
 find_package(Python3 REQUIRED COMPONENTS Interpreter)
 find_package(ament_cmake_gtest REQUIRED)
 ament_add_gtest(test_ik test/test_ik.cpp)
 target_link_libraries(test_ik luxi_ik)
 ament_target_dependencies(test_ik rclcpp luxi_interfaces luxi_core)
 target_compile_definitions(test_ik PRIVATE LUXI_ROOT="${LUXI_ROOT}")
 add_test(NAME motion_reference
   COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_motion_reference.py
     $<TARGET_FILE:arm_motion>
     ${LUXI_ROOT}/Project/robot_bringup/config/arms-kh.json
     ${LUXI_ROOT}/device/openarm/openarm_mujoco/v2/cell/cell.xml)
 set_tests_properties(motion_reference PROPERTIES TIMEOUT 20)
 add_test(NAME ik_session
   COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ik_session.py
     $<TARGET_FILE:arm_motion>
     ${LUXI_ROOT}/Project/robot_bringup/config/arms-kh.json
     ${LUXI_ROOT}/device/openarm/openarm_mujoco/v2/cell/cell.xml)
 set_tests_properties(ik_session PROPERTIES TIMEOUT 25)
endif()
ament_package()
