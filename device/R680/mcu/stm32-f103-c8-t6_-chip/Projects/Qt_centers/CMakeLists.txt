cmake_minimum_required(VERSION 3.21)

project(QtCenters VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(MACHINE_CONFIG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../excavator_controlled_servo/config")

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Qt6 6.5 REQUIRED COMPONENTS Core Widgets SerialPort)

add_library(qt_centers_protocol STATIC
    src/servo_protocol.cpp
    src/servo_protocol.h
)
target_link_libraries(qt_centers_protocol PUBLIC Qt6::Core)
target_include_directories(qt_centers_protocol PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")

add_library(qt_centers_config STATIC
    src/machine_config.cpp
    src/machine_config.h
)
target_link_libraries(qt_centers_config PUBLIC Qt6::Core)
target_include_directories(qt_centers_config PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")

add_library(qt_centers_ui STATIC
    src/main_window.cpp
    src/main_window.h
    src/serial_controller.cpp
    src/serial_controller.h
)
target_link_libraries(qt_centers_ui PUBLIC
    qt_centers_config
    qt_centers_protocol
    Qt6::Widgets
    Qt6::SerialPort
)
target_include_directories(qt_centers_ui PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")

add_executable(QtCenters WIN32 src/main.cpp)
target_link_libraries(QtCenters PRIVATE qt_centers_ui)
add_custom_command(TARGET QtCenters POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${MACHINE_CONFIG_DIR}"
            "$<TARGET_FILE_DIR:QtCenters>/config"
)

enable_testing()

add_executable(qt_centers_protocol_test
    tests/test_servo_protocol.cpp
)
target_link_libraries(qt_centers_protocol_test PRIVATE
    qt_centers_protocol
)
add_test(NAME qt_centers_protocol_test COMMAND qt_centers_protocol_test)

add_executable(qt_centers_config_test
    tests/test_machine_config.cpp
)
target_link_libraries(qt_centers_config_test PRIVATE qt_centers_config)
add_test(NAME qt_centers_config_test
    COMMAND qt_centers_config_test "${MACHINE_CONFIG_DIR}/machines.json"
)
add_test(NAME qt_centers_config2_test
    COMMAND qt_centers_config_test "${MACHINE_CONFIG_DIR}/config2.json" --config2-only
)

add_executable(qt_centers_ui_smoke
    tests/ui_smoke_test.cpp
)
target_link_libraries(qt_centers_ui_smoke PRIVATE qt_centers_ui)
add_test(NAME qt_centers_ui_smoke COMMAND qt_centers_ui_smoke)
set_tests_properties(qt_centers_ui_smoke PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
)

add_executable(qt_centers_serial_smoke
    tests/serial_smoke_test.cpp
)
target_link_libraries(qt_centers_serial_smoke PRIVATE
    qt_centers_protocol
    Qt6::Core
    Qt6::SerialPort
)
