cmake_minimum_required(VERSION 3.20)

project("F103C8T6_Chip_Workspace")
enable_language(C CXX ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

foreach(cmsis_var
    CMSIS_Dvendor
    CMSIS_Dname
    CMSIS_Dtz
    CMSIS_Dfpu
    CMSIS_Ddsp
    CMSIS_Dmve
    CMSIS_Dmpu
    CMSIS_Tcompiler
)
    if(DEFINED ${cmsis_var})
        set(_cmsis_${cmsis_var} "${${cmsis_var}}")
    endif()
endforeach()

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

if(NOT DEFINED PROJECT_BOARD)
    set(PROJECT_BOARD F103C8T6_Chip)
endif()

set(WORKSPACE_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(PROJECT_DIR ${WORKSPACE_ROOT}/Projects/${PROJECT_BOARD})

if(NOT EXISTS ${PROJECT_DIR}/CMakeLists.txt)
    message(FATAL_ERROR "Unknown PROJECT_BOARD '${PROJECT_BOARD}'. Expected project CMakeLists.txt in: ${PROJECT_DIR}")
endif()

option(ENABLE_LVGL "Enable common LVGL middleware for the active project" OFF)
option(ENABLE_ST7789_DISPLAY "Enable common ST7789 SPI display component for the active project" OFF)
option(ENABLE_ANALOG_INPUTS "Enable common ADC analog input peripheral for the active project" OFF)
option(ENABLE_NRF24L01 "Enable common NRF24L01 hardware SPI component for the active project" OFF)
option(ENABLE_KEYS "Enable common GPIO key peripheral for the active project" OFF)
option(ENABLE_FEETECH_STS3215 "Enable common Feetech STS3215 serial servo component for the active project" OFF)
option(ENABLE_DEBUG_UART "Enable common debug UART peripheral for the active project" OFF)
option(ENABLE_CAN "Enable common CAN peripheral for the active project" OFF)
option(ENABLE_PD42S1 "Enable common PD42S1 CAN stepper driver component" OFF)
option(ENABLE_ISL60PR "Enable common ISL60-PR Modbus RTU servo component" OFF)
option(ENABLE_POSITION_SENSORS "Enable common discrete position sensor peripheral" OFF)
option(ENABLE_BRAKE_RELAY "Enable common motor brake relay peripheral" OFF)
option(EXCAVATOR_CONTROL_KEY_SELF_TEST "Simulate controller drive keys once after boot" OFF)

set(cpu_PARAMS ${CPU_FLAGS})

get_filename_component(STM32_TOOLCHAIN_BIN_DIR "${CMAKE_C_COMPILER}" DIRECTORY)
find_program(STM32_SIZE_TOOL
    NAMES ${CMAKE_SIZE} arm-none-eabi-size
    HINTS "${STM32_TOOLCHAIN_BIN_DIR}"
    REQUIRED
)
find_program(STM32_OBJCOPY_TOOL
    NAMES ${CMAKE_OBJCOPY} arm-none-eabi-objcopy
    HINTS "${STM32_TOOLCHAIN_BIN_DIR}"
    REQUIRED
)

set(LVGL_ROOT ${WORKSPACE_ROOT}/Middlewares/LVGL/lvgl)
set(LVGL_PORT_ROOT ${WORKSPACE_ROOT}/Common/AppShared/LVGL)
set(ST7789_ROOT ${WORKSPACE_ROOT}/Common/BSP/Components/LCD_ST7789)
set(ANALOG_ROOT ${WORKSPACE_ROOT}/Common/BSP/Peripherals/Analog)
set(NRF24L01_ROOT ${WORKSPACE_ROOT}/Common/BSP/Components/NRF24L01)
set(KEY_ROOT ${WORKSPACE_ROOT}/Common/BSP/Peripherals/KEY)
set(FEETECH_STS3215_ROOT ${WORKSPACE_ROOT}/Common/BSP/Components/Feetech_STS3215)
set(DEBUG_UART_ROOT ${WORKSPACE_ROOT}/Common/BSP/Peripherals/UART)
set(CAN_ROOT ${WORKSPACE_ROOT}/Common/BSP/Peripherals/CAN)
set(PD42S1_ROOT ${WORKSPACE_ROOT}/Common/BSP/Components/PD42S1)
set(ISL60PR_ROOT ${WORKSPACE_ROOT}/Common/BSP/Components/ISL60PR)
set(POSITION_SENSOR_ROOT ${WORKSPACE_ROOT}/Common/BSP/Peripherals/PositionSensor)
set(BRAKE_RELAY_ROOT ${WORKSPACE_ROOT}/Common/BSP/Peripherals/BrakeRelay)

set(STM32F10X_STDPERIPH_SRCS
    ${WORKSPACE_ROOT}/Libraries/STM32F10x_StdPeriph_Driver/src/misc.c
    ${WORKSPACE_ROOT}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_adc.c
    ${WORKSPACE_ROOT}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_bkp.c
    ${WORKSPACE_ROOT}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_cec.c
    ${WORKSPACE_ROOT}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_crc.c
    ${WORKSPACE_ROOT}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_dac.c
    ${WORKSPACE_ROOT}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_dbgmcu.c
    ${WORKSPACE_ROOT}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_dma.c
    ${WORKSPACE_ROOT}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_exti.c
    ${WORKSPACE_ROOT}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_flash.c
    ${WORKSPACE_ROOT}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_fsmc.c
    ${WORKSPACE_ROOT}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_gpio.c
    ${WORKSPACE_ROOT}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_i2c.c
    ${WORKSPACE_ROOT}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_iwdg.c
    ${WORKSPACE_ROOT}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_pwr.c
    ${WORKSPACE_ROOT}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_rcc.c
    ${WORKSPACE_ROOT}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_rtc.c
    ${WORKSPACE_ROOT}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_sdio.c
    ${WORKSPACE_ROOT}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_spi.c
    ${WORKSPACE_ROOT}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_tim.c
    ${WORKSPACE_ROOT}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_usart.c
    ${WORKSPACE_ROOT}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_wwdg.c
)

set(FREERTOS_SRCS
    ${WORKSPACE_ROOT}/Middlewares/FreeRTOS-Kernel/list.c
    ${WORKSPACE_ROOT}/Middlewares/FreeRTOS-Kernel/queue.c
    ${WORKSPACE_ROOT}/Middlewares/FreeRTOS-Kernel/tasks.c
    ${WORKSPACE_ROOT}/Middlewares/FreeRTOS-Kernel/event_groups.c
    ${WORKSPACE_ROOT}/Middlewares/FreeRTOS-Kernel/stream_buffer.c
    ${WORKSPACE_ROOT}/Middlewares/FreeRTOS-Kernel/portable/GCC/ARM_CM3/port.c
    ${WORKSPACE_ROOT}/Middlewares/FreeRTOS-Kernel/portable/MemMang/heap_4.c
)

set(COMMON_SRCS
    ${WORKSPACE_ROOT}/Common/BSP/Board/bsp.c
    ${WORKSPACE_ROOT}/Common/BSP/Peripherals/LED/bsp_led.c
)

set(COMMON_INCS
    ${WORKSPACE_ROOT}/Common/BSP/Board
    ${WORKSPACE_ROOT}/Common/BSP/Peripherals/LED
    ${WORKSPACE_ROOT}/Libraries/CMSIS/CM3/CoreSupport
    ${WORKSPACE_ROOT}/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x
    ${WORKSPACE_ROOT}/Libraries/STM32F10x_StdPeriph_Driver/inc
    ${WORKSPACE_ROOT}/Middlewares/FreeRTOS-Kernel/include
    ${WORKSPACE_ROOT}/Middlewares/FreeRTOS-Kernel/portable/GCC/ARM_CM3
)

set(COMMON_DEFS
    STM32F10X_MD
    USE_STDPERIPH_DRIVER
)

if(ENABLE_LVGL)
    if(NOT EXISTS ${LVGL_ROOT}/lvgl.h)
        message(FATAL_ERROR "ENABLE_LVGL is ON, but LVGL source was not found in: ${LVGL_ROOT}")
    endif()

    file(GLOB_RECURSE LVGL_SRCS CONFIGURE_DEPENDS
        ${LVGL_ROOT}/src/*.c
    )

    list(APPEND COMMON_SRCS
        ${LVGL_SRCS}
        ${LVGL_PORT_ROOT}/lvgl_port.c
    )

    list(APPEND COMMON_INCS
        ${LVGL_ROOT}
        ${LVGL_ROOT}/src
        ${LVGL_PORT_ROOT}
    )

    list(APPEND COMMON_DEFS
        LV_CONF_INCLUDE_SIMPLE
    )
endif()

if(ENABLE_ST7789_DISPLAY)
    list(APPEND COMMON_SRCS
        ${ST7789_ROOT}/bsp_st7789.c
        ${ST7789_ROOT}/bsp_st7789_font.c
    )

    list(APPEND COMMON_INCS
        ${ST7789_ROOT}
    )
endif()

if(ENABLE_ANALOG_INPUTS)
    list(APPEND COMMON_SRCS
        ${ANALOG_ROOT}/bsp_analog.c
    )

    list(APPEND COMMON_INCS
        ${ANALOG_ROOT}
    )
endif()

if(ENABLE_NRF24L01)
    list(APPEND COMMON_SRCS
        ${NRF24L01_ROOT}/bsp_nrf24l01.c
    )

    list(APPEND COMMON_INCS
        ${NRF24L01_ROOT}
    )
endif()

if(ENABLE_KEYS)
    list(APPEND COMMON_SRCS
        ${KEY_ROOT}/bsp_key.c
    )

    list(APPEND COMMON_INCS
        ${KEY_ROOT}
    )
endif()

if(ENABLE_FEETECH_STS3215)
    list(APPEND COMMON_SRCS
        ${FEETECH_STS3215_ROOT}/bsp_sts3215.c
    )

    list(APPEND COMMON_INCS
        ${FEETECH_STS3215_ROOT}
    )
endif()

if(ENABLE_DEBUG_UART)
    list(APPEND COMMON_SRCS
        ${DEBUG_UART_ROOT}/bsp_uart.c
    )

    list(APPEND COMMON_INCS
        ${DEBUG_UART_ROOT}
    )
endif()

if(ENABLE_CAN)
    list(APPEND COMMON_SRCS
        ${WORKSPACE_ROOT}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_can.c
        ${CAN_ROOT}/bsp_can.c
    )

    list(APPEND COMMON_INCS
        ${CAN_ROOT}
    )
endif()

if(ENABLE_PD42S1)
    if(NOT ENABLE_CAN)
        message(FATAL_ERROR "ENABLE_PD42S1 requires ENABLE_CAN=ON")
    endif()

    list(APPEND COMMON_SRCS
        ${PD42S1_ROOT}/bsp_pd42s1.c
    )

    list(APPEND COMMON_INCS
        ${PD42S1_ROOT}
    )
endif()

if(ENABLE_ISL60PR)
    list(APPEND COMMON_SRCS
        ${ISL60PR_ROOT}/bsp_isl60pr.c
    )

    list(APPEND COMMON_INCS
        ${ISL60PR_ROOT}
    )
endif()

if(ENABLE_POSITION_SENSORS)
    list(APPEND COMMON_SRCS
        ${POSITION_SENSOR_ROOT}/bsp_position_sensor.c
    )
    list(APPEND COMMON_INCS
        ${POSITION_SENSOR_ROOT}
    )
    list(APPEND COMMON_DEFS
        BSP_POSITION_SENSORS_ENABLED
    )
endif()

if(ENABLE_BRAKE_RELAY)
    list(APPEND COMMON_SRCS
        ${BRAKE_RELAY_ROOT}/bsp_brake_relay.c
    )
    list(APPEND COMMON_INCS
        ${BRAKE_RELAY_ROOT}
    )
    list(APPEND COMMON_DEFS
        BSP_BRAKE_RELAY_ENABLED
    )
endif()

add_subdirectory(${PROJECT_DIR})
