#include "position_monitor_task.h"

#include <stdbool.h>
#include <stdint.h>

#include "bsp_position_sensor.h"
#include "servo_app_config.h"
#include "servo_jog_task.h"

#define CENTER_SENSOR_MASK  1U
#define CENTER_POSITION     3U

typedef struct
{
    bool stable_active;
    bool candidate_active;
    uint16_t candidate_time_ms;
} PositionSensorDebounce;

static PositionSensorDebounce center_sensor;

static bool PositionMonitor_Debounce(bool raw_active)
{
    uint16_t debounce_ms;

    if (raw_active != center_sensor.candidate_active)
    {
        center_sensor.candidate_active = raw_active;
        center_sensor.candidate_time_ms = 0U;
    }
    else
    {
        debounce_ms = raw_active ? SERVO_POSITION_ASSERT_DEBOUNCE_MS
                                 : SERVO_POSITION_RELEASE_DEBOUNCE_MS;
        if (center_sensor.candidate_time_ms < debounce_ms)
        {
            center_sensor.candidate_time_ms = (uint16_t)(
                center_sensor.candidate_time_ms + SERVO_POSITION_SCAN_PERIOD_MS);
            if (center_sensor.candidate_time_ms >= debounce_ms)
                center_sensor.stable_active = raw_active;
        }
    }

    return center_sensor.stable_active;
}

static void PositionMonitor_Task(void *argument)
{
    uint8_t previous_raw_mask = 0U;

    (void)argument;

    for (;;)
    {
        uint8_t raw_mask = BSP_PositionSensor_ReadMask() & CENTER_SENSOR_MASK;
        uint8_t stable_mask = PositionMonitor_Debounce(raw_mask != 0U) ?
            CENTER_SENSOR_MASK : 0U;

        if (raw_mask != 0U && previous_raw_mask == 0U)
            g_servo_jog_diagnostics.position_sensor_raw_events++;

        g_servo_jog_diagnostics.position_sensor_raw_mask = raw_mask;
        g_servo_jog_diagnostics.position_sensor_mask = stable_mask;
        g_servo_jog_diagnostics.position_sensor_fault_active = 0U;
        g_servo_jog_diagnostics.actual_position =
            stable_mask != 0U ? CENTER_POSITION : 0U;

        if (stable_mask != 0U &&
            g_servo_jog_diagnostics.last_position != CENTER_POSITION)
        {
            g_servo_jog_diagnostics.last_position = CENTER_POSITION;
            g_servo_jog_diagnostics.position_change_count++;
        }

        previous_raw_mask = raw_mask;
        vTaskDelay(pdMS_TO_TICKS(SERVO_POSITION_SCAN_PERIOD_MS));
    }
}

void PositionMonitor_TaskCreate(void)
{
    (void)xTaskCreate(PositionMonitor_Task,
                      POSITION_MONITOR_TASK_NAME,
                      POSITION_MONITOR_TASK_STACK_WORDS,
                      NULL,
                      POSITION_MONITOR_TASK_PRIORITY,
                      NULL);
}
