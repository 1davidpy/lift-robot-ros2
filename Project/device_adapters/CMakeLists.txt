cmake_minimum_required(VERSION 3.16)
project(luxi_device_adapters)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(luxi_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(luxi_core REQUIRED)

find_package(OpenCV REQUIRED)
file(GLOB_RECURSE OPENARM_SOURCES "${LUXI_ROOT}/device/openarm/openarm_can/src/*.cpp")
add_library(luxi_openarm_can STATIC ${OPENARM_SOURCES})
target_include_directories(luxi_openarm_can PUBLIC "${LUXI_ROOT}/device/openarm/openarm_can/include")
add_executable(r680_adapter src/r680_adapter.cpp)
target_include_directories(r680_adapter PRIVATE include)
ament_target_dependencies(r680_adapter rclcpp luxi_interfaces luxi_core std_msgs sensor_msgs geometry_msgs)
install(TARGETS r680_adapter DESTINATION lib/${PROJECT_NAME})
add_executable(lift_adapter src/lift_adapter.cpp)
target_include_directories(lift_adapter PRIVATE include)
ament_target_dependencies(lift_adapter rclcpp luxi_interfaces luxi_core std_msgs)
install(TARGETS lift_adapter DESTINATION lib/${PROJECT_NAME})
add_executable(openarm_adapter src/openarm_adapter.cpp)
target_link_libraries(openarm_adapter luxi_openarm_can)
ament_target_dependencies(openarm_adapter rclcpp luxi_interfaces luxi_core std_msgs)
add_executable(hikrobot_camera src/hikrobot_camera.cpp)
target_include_directories(hikrobot_camera PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(hikrobot_camera ${OpenCV_LIBS} dl)
ament_target_dependencies(hikrobot_camera rclcpp luxi_interfaces luxi_core sensor_msgs)
add_executable(realsense_camera src/realsense_camera.cpp)
target_include_directories(realsense_camera PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(realsense_camera ${OpenCV_LIBS})
ament_target_dependencies(realsense_camera rclcpp luxi_interfaces luxi_core sensor_msgs)
add_executable(kh_can_setup src/kh_can_setup.cpp)
target_include_directories(kh_can_setup PRIVATE include)
install(TARGETS kh_can_setup openarm_adapter hikrobot_camera realsense_camera DESTINATION lib/${PROJECT_NAME})
find_package(realsense2 QUIET CONFIG)
add_executable(realsense_depth src/realsense_depth.cpp)
target_include_directories(realsense_depth PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(realsense_depth dl ${OpenCV_LIBS})
ament_target_dependencies(realsense_depth rclcpp luxi_interfaces luxi_core sensor_msgs)
if(realsense2_FOUND)
 target_compile_definitions(realsense_depth PRIVATE LUXI_HAVE_REALSENSE=1)
 add_library(luxi_realsense_depth SHARED src/realsense_depth_backend.cpp)
 target_include_directories(luxi_realsense_depth PRIVATE include)
 target_link_libraries(luxi_realsense_depth PRIVATE realsense2::realsense2)
 set_target_properties(luxi_realsense_depth PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
 install(TARGETS luxi_realsense_depth LIBRARY DESTINATION lib/${PROJECT_NAME})
else()
 message(WARNING "librealsense2 unavailable: realsense_depth supports mock only; run tool/build_depth_sdk.sh")
endif()
install(TARGETS realsense_depth DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
 find_package(Python3 REQUIRED COMPONENTS Interpreter)
 add_executable(test_static_position_compensation test/test_static_position_compensation.cpp)
 add_test(NAME static_position_compensation COMMAND test_static_position_compensation)
 add_test(NAME openarm_static_load
   COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_openarm_static_load.py
     $<TARGET_FILE:openarm_adapter>
     ${LUXI_ROOT}/build/packages/luxi_control_manager/control_manager
     ${LUXI_ROOT}/Project/robot_bringup/config/arms-kh.json)
 set_tests_properties(openarm_static_load PROPERTIES TIMEOUT 75)
 add_executable(test_openarm_monitor_readonly test/test_openarm_monitor_readonly.cpp)
 add_test(NAME openarm_monitor_readonly COMMAND test_openarm_monitor_readonly)
 add_test(NAME openarm_manual_pose
   COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_openarm_manual_pose.py
     $<TARGET_FILE:openarm_adapter>
     ${LUXI_ROOT}/Project/robot_bringup/config/arms-kh.json)
 set_tests_properties(openarm_manual_pose PROPERTIES TIMEOUT 40)
 add_test(NAME openarm_joint_target
   COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_openarm_joint_target.py
     $<TARGET_FILE:openarm_adapter>
     ${LUXI_ROOT}/build/packages/luxi_control_manager/control_manager
     ${LUXI_ROOT}/build/install/luxi_web_ctrl/lib/luxi_web_ctrl/web_ctrl
     ${LUXI_ROOT}/Project/robot_bringup/config/arms-kh.json)
 set_tests_properties(openarm_joint_target PROPERTIES TIMEOUT 75)
 add_test(NAME openarm_continuous_handover
   COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_openarm_continuous_handover.py
     $<TARGET_FILE:openarm_adapter>
     ${LUXI_ROOT}/build/packages/luxi_control_manager/control_manager
     ${LUXI_ROOT}/build/install/luxi_web_ctrl/lib/luxi_web_ctrl/web_ctrl
     ${LUXI_ROOT}/Project/robot_bringup/config/arms-kh.json)
 set_tests_properties(openarm_continuous_handover PROPERTIES TIMEOUT 60)
 add_test(NAME teleop_status_rates
   COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_status_rates.py
     $<TARGET_FILE:openarm_adapter>
     ${LUXI_ROOT}/build/packages/luxi_control_manager/control_manager
     ${LUXI_ROOT}/Project/robot_bringup/config/arms-teleop.json)
 set_tests_properties(teleop_status_rates PROPERTIES TIMEOUT 30)

 add_test(NAME depth_output
   COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_depth_output.py
     $<TARGET_FILE:realsense_depth>)
 set_tests_properties(depth_output PROPERTIES TIMEOUT 25)
 add_test(NAME lift_network_recovery
   COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_lift_network_recovery.py
     $<TARGET_FILE:lift_adapter>)
 set_tests_properties(lift_network_recovery PROPERTIES TIMEOUT 75)
 add_test(NAME lift_position_invalid
   COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_lift_position_invalid.py
     $<TARGET_FILE:lift_adapter>)
 set_tests_properties(lift_position_invalid PROPERTIES TIMEOUT 75)
 add_test(NAME base_quest
   COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_base_quest.py
     $<TARGET_FILE:r680_adapter>)
 set_tests_properties(base_quest PROPERTIES TIMEOUT 60)
 find_package(PkgConfig REQUIRED)
 pkg_check_modules(HTTPLIB REQUIRED IMPORTED_TARGET cpp-httplib)
 add_executable(r680_recording_check test/r680_recording_check.cpp)
 target_link_libraries(r680_recording_check PkgConfig::HTTPLIB)
 install(TARGETS r680_recording_check DESTINATION lib/${PROJECT_NAME})
 add_executable(r680_socket_probe test/r680_socket_probe.cpp)
 target_include_directories(r680_socket_probe PRIVATE include)
 install(TARGETS r680_socket_probe DESTINATION lib/${PROJECT_NAME})
 add_executable(r680_integration_probe test/r680_integration_probe.cpp)
 ament_target_dependencies(r680_integration_probe rclcpp luxi_core luxi_interfaces std_msgs)
 install(TARGETS r680_integration_probe DESTINATION lib/${PROJECT_NAME})
 add_test(NAME r680_integration COMMAND bash "${LUXI_ROOT}/tool/r680_integration.sh")
 set_tests_properties(r680_integration PROPERTIES TIMEOUT 60)
 add_executable(socketcan_probe test/socketcan_probe.cpp)
 target_link_libraries(socketcan_probe luxi_openarm_can)
 install(TARGETS socketcan_probe DESTINATION lib/${PROJECT_NAME})
 find_package(ament_cmake_gtest REQUIRED)
 ament_add_gtest(test_lift_uart test/test_lift_uart.cpp)
 target_include_directories(test_lift_uart PRIVATE include)
 ament_target_dependencies(test_lift_uart rclcpp luxi_core luxi_interfaces std_msgs)
 ament_add_gtest(test_r680 test/test_r680.cpp)
 target_include_directories(test_r680 PRIVATE include)
 ament_add_gtest(test_kh_can test/test_kh_can.cpp)
 target_include_directories(test_kh_can PRIVATE include)
 add_library(fake_mv3d SHARED test/fake_mv3d.cpp)
 target_include_directories(fake_mv3d PRIVATE include ${OpenCV_INCLUDE_DIRS})
 ament_add_gtest(test_camera test/test_camera.cpp)
 target_include_directories(test_camera PRIVATE include ${OpenCV_INCLUDE_DIRS})
 target_link_libraries(test_camera ${OpenCV_LIBS} dl)
 target_compile_definitions(test_camera PRIVATE FAKE_SDK="$<TARGET_FILE:fake_mv3d>")
 add_dependencies(test_camera fake_mv3d)
endif()
ament_package()
