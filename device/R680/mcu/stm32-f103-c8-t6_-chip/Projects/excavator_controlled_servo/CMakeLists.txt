set(TARGET_NAME excavator_controlled_servo)
set(PROJECT_ROOT ${CMAKE_CURRENT_LIST_DIR})

set(MACHINE_CONFIG_FILE "${PROJECT_ROOT}/config/config2.json")
file(READ "${MACHINE_CONFIG_FILE}" MACHINE_CONFIG_JSON)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${MACHINE_CONFIG_FILE}")
string(JSON MACHINE_SCHEMA GET "${MACHINE_CONFIG_JSON}" schema_version)
if(NOT MACHINE_SCHEMA EQUAL 2)
    message(FATAL_ERROR "Seven-servo firmware requires machine schema_version 2")
endif()
string(JSON ACTIVE_MACHINE_PROFILE GET "${MACHINE_CONFIG_JSON}" active_profile)
string(JSON MACHINE_PROFILE_COUNT LENGTH "${MACHINE_CONFIG_JSON}" profiles)
math(EXPR MACHINE_PROFILE_LAST "${MACHINE_PROFILE_COUNT} - 1")
set(ACTIVE_MACHINE_PROFILE_INDEX -1)
foreach(PROFILE_INDEX RANGE 0 ${MACHINE_PROFILE_LAST})
    string(JSON PROFILE_ID GET "${MACHINE_CONFIG_JSON}" profiles ${PROFILE_INDEX} id)
    if("${PROFILE_ID}" STREQUAL "${ACTIVE_MACHINE_PROFILE}")
        set(ACTIVE_MACHINE_PROFILE_INDEX ${PROFILE_INDEX})
        break()
    endif()
endforeach()
if(ACTIVE_MACHINE_PROFILE_INDEX EQUAL -1)
    message(FATAL_ERROR "active_profile '${ACTIVE_MACHINE_PROFILE}' was not found in ${MACHINE_CONFIG_FILE}")
endif()

string(JSON EXCAVATOR_BOOM_POSITION_MIN GET "${MACHINE_CONFIG_JSON}" profiles ${ACTIVE_MACHINE_PROFILE_INDEX} limits 0 minimum)
string(JSON EXCAVATOR_BOOM_POSITION_MAX GET "${MACHINE_CONFIG_JSON}" profiles ${ACTIVE_MACHINE_PROFILE_INDEX} limits 0 maximum)
string(JSON EXCAVATOR_ARM_POSITION_MIN GET "${MACHINE_CONFIG_JSON}" profiles ${ACTIVE_MACHINE_PROFILE_INDEX} limits 1 minimum)
string(JSON EXCAVATOR_ARM_POSITION_MAX GET "${MACHINE_CONFIG_JSON}" profiles ${ACTIVE_MACHINE_PROFILE_INDEX} limits 1 maximum)
string(JSON EXCAVATOR_PITCH_POSITION_MIN GET "${MACHINE_CONFIG_JSON}" profiles ${ACTIVE_MACHINE_PROFILE_INDEX} limits 2 minimum)
string(JSON EXCAVATOR_PITCH_POSITION_MAX GET "${MACHINE_CONFIG_JSON}" profiles ${ACTIVE_MACHINE_PROFILE_INDEX} limits 2 maximum)
string(JSON EXCAVATOR_GRIPPER_POSITION_MIN GET "${MACHINE_CONFIG_JSON}" profiles ${ACTIVE_MACHINE_PROFILE_INDEX} limits 3 minimum)
string(JSON EXCAVATOR_GRIPPER_POSITION_MAX GET "${MACHINE_CONFIG_JSON}" profiles ${ACTIVE_MACHINE_PROFILE_INDEX} limits 3 maximum)
set(EXPECTED_AXES 0 1 2 4)
set(EXPECTED_IDS 1 2 3 8)
set(ENABLED_SYMBOLS BOOM ARM PITCH GRIPPER)
string(JSON LIMIT_COUNT LENGTH "${MACHINE_CONFIG_JSON}" profiles ${ACTIVE_MACHINE_PROFILE_INDEX} limits)
if(NOT LIMIT_COUNT EQUAL 4)
    message(FATAL_ERROR "Machine profile must contain limits for IDs 1, 2, 3, 8")
endif()
foreach(INDEX RANGE 0 3)
    string(JSON AXIS GET "${MACHINE_CONFIG_JSON}" profiles ${ACTIVE_MACHINE_PROFILE_INDEX} limits ${INDEX} axis)
    string(JSON SERVO_ID GET "${MACHINE_CONFIG_JSON}" profiles ${ACTIVE_MACHINE_PROFILE_INDEX} limits ${INDEX} servo_id)
    string(JSON MINIMUM GET "${MACHINE_CONFIG_JSON}" profiles ${ACTIVE_MACHINE_PROFILE_INDEX} limits ${INDEX} minimum)
    string(JSON MAXIMUM GET "${MACHINE_CONFIG_JSON}" profiles ${ACTIVE_MACHINE_PROFILE_INDEX} limits ${INDEX} maximum)
    string(JSON ENABLED GET "${MACHINE_CONFIG_JSON}" profiles ${ACTIVE_MACHINE_PROFILE_INDEX} limits ${INDEX} enabled)
    string(JSON ENABLED_TYPE TYPE "${MACHINE_CONFIG_JSON}" profiles ${ACTIVE_MACHINE_PROFILE_INDEX} limits ${INDEX} enabled)
    list(GET ENABLED_SYMBOLS ${INDEX} SYMBOL)
    string(JSON REFERENCE_MODE GET "${MACHINE_CONFIG_JSON}" profiles ${ACTIVE_MACHINE_PROFILE_INDEX} limits ${INDEX} reference_mode)
    string(JSON ENCODER_OFFSET GET "${MACHINE_CONFIG_JSON}" profiles ${ACTIVE_MACHINE_PROFILE_INDEX} limits ${INDEX} encoder_offset)
    if(REFERENCE_MODE LESS 0 OR REFERENCE_MODE GREATER 1 OR ENCODER_OFFSET LESS 0 OR ENCODER_OFFSET GREATER 2047 OR
       (REFERENCE_MODE EQUAL 1 AND MINIMUM LESS ENCODER_OFFSET))
        message(FATAL_ERROR "Invalid position reference at index ${INDEX}")
    endif()
    set(EXCAVATOR_${SYMBOL}_REFERENCE_MODE ${REFERENCE_MODE})
    set(EXCAVATOR_${SYMBOL}_ENCODER_OFFSET ${ENCODER_OFFSET})
    set(EXCAVATOR_${SYMBOL}_LIMITS_ENABLED 0)
    if(ENABLED)
        set(EXCAVATOR_${SYMBOL}_LIMITS_ENABLED 1)
    endif()
    list(GET EXPECTED_AXES ${INDEX} EXPECTED_AXIS)
    list(GET EXPECTED_IDS ${INDEX} EXPECTED_ID)
    if(NOT AXIS EQUAL EXPECTED_AXIS OR NOT SERVO_ID EQUAL EXPECTED_ID OR
       MINIMUM LESS 0 OR MAXIMUM GREATER 4095 OR MINIMUM GREATER_EQUAL MAXIMUM OR
       NOT ENABLED_TYPE STREQUAL "BOOLEAN")
        message(FATAL_ERROR "Invalid position axis in machine profile at index ${INDEX}")
    endif()
endforeach()

if(EXCAVATOR_BOOM_POSITION_MIN GREATER_EQUAL EXCAVATOR_BOOM_POSITION_MAX OR
   EXCAVATOR_ARM_POSITION_MIN GREATER_EQUAL EXCAVATOR_ARM_POSITION_MAX OR
   EXCAVATOR_PITCH_POSITION_MIN GREATER_EQUAL EXCAVATOR_PITCH_POSITION_MAX)
    message(FATAL_ERROR "Each machine limit minimum must be smaller than its maximum")
endif()

set(GENERATED_CONFIG_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${GENERATED_CONFIG_DIR}")
configure_file(
    "${PROJECT_ROOT}/Inc/excavator_machine_limits.h.in"
    "${GENERATED_CONFIG_DIR}/excavator_machine_limits.h"
    @ONLY
)

set(PROJECT_SRCS
    ${PROJECT_ROOT}/Src/main.c
    ${PROJECT_ROOT}/Src/excavator_servo_config.c
    ${PROJECT_ROOT}/Src/syscall.c
    ${PROJECT_ROOT}/Src/sysmem.c
    ${PROJECT_ROOT}/App/RTOS/freertos.c
    ${PROJECT_ROOT}/App/Protocol/excavator_protocol.c
    ${PROJECT_ROOT}/App/Protocol/excavator_position_control.c
    ${PROJECT_ROOT}/App/Tasks/servo_control_task.c
    ${PROJECT_ROOT}/App/Tasks/nrf_control_rx_task.c
    ${PROJECT_ROOT}/Startup/startup_stm32f103xx.S
    ${WORKSPACE_ROOT}/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.c
)

set(PROJECT_INCS
    ${GENERATED_CONFIG_DIR}
    ${PROJECT_ROOT}/Inc
    ${PROJECT_ROOT}/App/RTOS
    ${PROJECT_ROOT}/App/Protocol
    ${PROJECT_ROOT}/App/Tasks
    ${COMMON_INCS}
)

set(LINKER_SCRIPT ${PROJECT_ROOT}/Linker/stm32f103x8_flash.ld)

add_executable(${TARGET_NAME}
    ${PROJECT_SRCS}
    ${COMMON_SRCS}
    ${STM32F10X_STDPERIPH_SRCS}
    ${FREERTOS_SRCS}
)

target_include_directories(${TARGET_NAME} PRIVATE ${PROJECT_INCS})

target_compile_definitions(${TARGET_NAME} PRIVATE
    ${COMMON_DEFS}
    $<$<CONFIG:Debug>:DEBUG>
)

target_compile_options(${TARGET_NAME} PRIVATE
    ${cpu_PARAMS}
    $<$<COMPILE_LANGUAGE:C>:-include${PROJECT_ROOT}/Inc/stm32f10x_conf.h>
    -fstack-usage
    -Wall
    -Wextra
    -Wpedantic
    -Wno-unused-parameter
    $<$<COMPILE_LANGUAGE:ASM>:-x assembler-with-cpp -MMD -MP>
    $<$<CONFIG:Debug>:-Og -g3 -ggdb>
    $<$<CONFIG:Release>:-Os>
)

target_link_options(${TARGET_NAME} PRIVATE
    -T${LINKER_SCRIPT}
    ${cpu_PARAMS}
    -Wl,-Map=${TARGET_NAME}.map
    --specs=nosys.specs
    -Wl,--start-group
    -lc
    -lm
    -Wl,--end-group
    -Wl,-z,max-page-size=8
    -Wl,--print-memory-usage
)

set_target_properties(${TARGET_NAME} PROPERTIES
    LINK_DEPENDS ${LINKER_SCRIPT}
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${STM32_SIZE_TOOL} $<TARGET_FILE:${TARGET_NAME}>
    COMMAND ${STM32_OBJCOPY_TOOL} -O ihex $<TARGET_FILE:${TARGET_NAME}> $<TARGET_FILE_DIR:${TARGET_NAME}>/${TARGET_NAME}.hex
    COMMAND ${STM32_OBJCOPY_TOOL} -O binary $<TARGET_FILE:${TARGET_NAME}> $<TARGET_FILE_DIR:${TARGET_NAME}>/${TARGET_NAME}.bin
)
