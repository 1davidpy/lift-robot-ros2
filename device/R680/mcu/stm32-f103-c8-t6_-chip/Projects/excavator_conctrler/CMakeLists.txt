set(TARGET_NAME excavator_conctrler)
set(PROJECT_ROOT ${CMAKE_CURRENT_LIST_DIR})

set(PROJECT_SRCS
    ${PROJECT_ROOT}/Src/main.c
    ${PROJECT_ROOT}/Src/syscall.c
    ${PROJECT_ROOT}/Src/sysmem.c
    ${PROJECT_ROOT}/App/RTOS/freertos.c
    ${PROJECT_ROOT}/App/Protocol/excavator_protocol.c
    ${PROJECT_ROOT}/App/Tasks/system_led_task.c
    ${PROJECT_ROOT}/App/Tasks/display_task.c
    ${PROJECT_ROOT}/App/Tasks/input_adc_task.c
    ${PROJECT_ROOT}/App/Tasks/key_task.c
    ${PROJECT_ROOT}/App/Tasks/nrf_control_tx_task.c
    ${PROJECT_ROOT}/App/Tasks/wireless_task.c
    ${PROJECT_ROOT}/Startup/startup_stm32f103xx.S
    ${WORKSPACE_ROOT}/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.c
)

set(PROJECT_INCS
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
    $<$<BOOL:${EXCAVATOR_CONTROL_KEY_SELF_TEST}>:EXCAVATOR_CONTROL_KEY_SELF_TEST>
    $<$<CONFIG:Debug>:DEBUG>
)

target_compile_options(${TARGET_NAME} PRIVATE
    ${cpu_PARAMS}
    -fstack-usage
    -Wall
    -Wextra
    -Wpedantic
    -Wno-unused-parameter
    $<$<COMPILE_LANGUAGE:ASM>:-x assembler-with-cpp -MMD -MP>
    $<$<CONFIG:Debug>:-O0 -g3 -ggdb>
    $<$<CONFIG:Release>:-Os>
)

target_link_options(${TARGET_NAME} PRIVATE
    -T${LINKER_SCRIPT}
    ${cpu_PARAMS}
    -Wl,-Map=${TARGET_NAME}.map
    -u _printf_float
    --specs=nosys.specs
    -Wl,--start-group
    -lc
    -lm
    -lstdc++
    -lsupc++
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
