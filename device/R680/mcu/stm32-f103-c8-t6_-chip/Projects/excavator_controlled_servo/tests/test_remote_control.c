/* Reuse simulated BSP/RTOS IO while executing the Config2 task implementation. */
#define main previous_stage_test_main
#include "test_servo_calibration.c"
#undef main

static uint16_t reference_goal(uint8_t id)
{
    return (uint16_t)(position_goals[id] + (id <= 3 ? 85 : 0));
}

static void test_boundaries(void)
{
    const uint16_t bounds[4][2] = {{1001, 2950}, {2223, 4076}, {1547, 3609}, {172, 1292}};
    for (unsigned axis = 0; axis < 4; axis++)
        for (uint16_t current = 0; current <= 4095; current++)
            for (uint8_t step = 1; step <= 10; step++)
                for (int8_t direction = -1; direction <= 1; direction += 2)
                {
                    uint16_t target;
                    if (ExcavatorPosition_NextTarget(current, bounds[axis][0], bounds[axis][1], direction, step, &target))
                    {
                        assert(ExcavatorPosition_TargetAllowed(current, target, bounds[axis][0], bounds[axis][1], step));
                        assert((direction > 0 && target > current) || (direction < 0 && target < current));
                        if (current < bounds[axis][0]) assert(direction > 0 && target <= bounds[axis][0]);
                        else if (current > bounds[axis][1]) assert(direction < 0 && target >= bounds[axis][1]);
                        else assert(target >= bounds[axis][0] && target <= bounds[axis][1]);
                    }
                }
    uint16_t target;
    assert(ExcavatorPosition_NextTarget(2143, 2223, 4076, 1, 2, &target) && target == 2145);
    assert(!ExcavatorPosition_NextTarget(2143, 2223, 4076, -1, 2, &target));
    assert(ExcavatorPosition_NextTarget(1291, 172, 1292, 1, 2, &target) && target == 1292);
    assert(!ExcavatorPosition_NextTarget(1292, 172, 1292, 1, 2, &target));
    assert(ExcavatorPosition_TargetAllowed(2222, 2224, 2223, 4076, 2));
    assert(!ExcavatorPosition_TargetAllowed(2222, 2220, 2223, 4076, 2));
    assert(ExcavatorPosition_TargetAllowed(1293, 1291, 172, 1292, 2));
    assert(!ExcavatorPosition_TargetAllowed(1293, 1295, 172, 1292, 2));
}

static void arm_neutral(void)
{
    ExcavatorVelocityCmd_t neutral = {0};
    ticks += 1;
    ServoControl_HandleWireless(&neutral);
    for (unsigned i = 0; i < 20; i++)
    {
        if (i < 19) assert(!remote_armed);
        ticks += 50;
        ServoControl_HandleWireless(&neutral);
    }
    assert(remote_armed && g_chassis_armed && radio_armed);
}

static void test_continuous_follow(void)
{
    const uint8_t ids[4] = {1, 2, 3, 8};
    const uint16_t references[4] = {1500, 2143, 2700, 1000};
    for (unsigned i = 0; i < 4; i++)
    {
        const uint8_t id = ids[i];
        feedback[id] = references[i] - (id <= 3 ? 85 : 0);
        position_goals[id] = feedback[id];
        modes[id] = 0; torque[id] = 1;
    }
    ServoControl_LoadParameters();
    ServoControl_UpdateFeedback();
    assert(remote_ready_mask == 23);
    arm_neutral();
    ExcavatorVelocityCmd_t input = {.boom = -10, .arm = 10};
    for (unsigned i = 0; i < 900; i++)
    {
        ticks += 20;
        ServoControl_HandleWireless(&input);
        ServoControl_UpdateMotion();
        for (unsigned id = 1; id <= 2; id++)
        {
            assert(position_goals[id] >= feedback[id] && position_goals[id] - feedback[id] <= 96);
            /* A plant with 16-count deadband exposes the former 8-count stall. */
            if (position_goals[id] - feedback[id] > 16) feedback[id] += 2;
        }
        assert(reference_goal(1) <= 2950 && reference_goal(2) <= 4076);
    }
    assert(feedback[1] + 85 > 2900 && feedback[2] + 85 > 2223);
    ExcavatorVelocityCmd_t neutral = {0};
    ServoControl_HandleWireless(&neutral);
    assert(position_goals[1] == feedback[1] && position_goals[2] == feedback[2]);
    input.boom = 10; input.arm = 0;
    for (unsigned i = 0; i < 1100; i++)
    {
        ticks += 20;
        ServoControl_HandleWireless(&input);
        ServoControl_UpdateMotion();
        assert(position_goals[1] <= feedback[1] && feedback[1] - position_goals[1] <= 96);
        if (feedback[1] - position_goals[1] > 16) feedback[1] -= 2;
        assert(reference_goal(1) >= 1001);
    }
    assert(feedback[1] + 85 < 1050);
    ServoControl_HandleWireless(&neutral);
    assert(position_goals[1] == feedback[1]);
    const unsigned held_writes = position_writes;
    ticks += 201;
    ServoControl_UpdateMotion();
    assert(!remote_armed && !g_chassis_armed && position_writes == held_writes);
}

static void test_loaded_hold_and_lift(void)
{
    feedback[1] = 1800 - 85;
    position_goals[1] = feedback[1] - 11;
    ServoControl_LoadParameters();
    ServoControl_UpdateFeedback();
    assert(remote_ready_mask == 23);
    position_goals[1] = feedback[1] - 85;
    ServoControl_LoadParameters();
    ServoControl_UpdateFeedback();
    assert((remote_ready_mask & 1) == 0); /* The old center is still rejected. */
    position_goals[1] = feedback[1] - 11;
    ServoControl_LoadParameters();
    ServoControl_UpdateFeedback();
    arm_neutral();
    ExcavatorVelocityCmd_t raise = {.boom = 10};
    const uint16_t before = feedback[1];
    for (unsigned i = 0; i < 60; i++)
    {
        ticks += 20;
        ServoControl_HandleWireless(&raise);
        ServoControl_UpdateMotion();
        assert(feedback[1] - position_goals[1] <= 96);
        /* A loaded plant requiring more than 30 counts must still progress. */
        if (feedback[1] - position_goals[1] > 64) feedback[1] -= 2;
    }
    assert(before - feedback[1] > 40 && last_position.speed == 250);
    ExcavatorVelocityCmd_t neutral = {0};
    ServoControl_HandleWireless(&neutral);
    assert(position_goals[1] == feedback[1]);
}

static void test_host_control(void)
{
    const uint8_t ids[4] = {1, 2, 3, 8};
    const uint16_t positions[4] = {1800, 2400, 2700, 1000};
    for (unsigned i = 0; i < 4; i++)
    {
        feedback[ids[i]] = positions[i] - (ids[i] <= 3 ? 85 : 0);
        position_goals[ids[i]] = feedback[ids[i]];
        modes[ids[i]] = 0; torque[ids[i]] = 1;
    }
    for (unsigned id = 4; id <= 6; id++) { modes[id] = 1; torque[id] = 1; }
    ServoControl_LoadParameters();
    ServoControl_UpdateFeedback();
    assert(remote_ready_mask == 23);
    command("CAPS", "CAPS HOST_CONTROL_V2\r\n");
    command("HOST_CONTROL 6 6 0 0 0 0 0", "ERR HOST_CONTROL\r\n");
    const unsigned before_modes = mode_writes;
    command("HOST_BEGIN", "OK HOST_BEGIN\r\n");
    assert(host_owned && host_streaming && !remote_armed && !g_chassis_armed);
    assert(mode_writes == before_modes); /* Ownership never prepares/unloads joints. */
    command("HOST_CONTROL 6 6 0 0 0 0 0", "OK HOST_CONTROL\r\n");
    assert(!g_chassis_armed && goals[5] == 0 && goals[6] == 0);
    for (unsigned i = 0; i < 24; i++)
    {
        ticks += 50;
        command("HOST_CONTROL 0 0 0 0 0 0 0", "OK HOST_CONTROL\r\n");
    }
    assert(remote_armed && g_chassis_armed);
    command("HOST_CONTROL 6 6 0 0 0 0 0", "OK HOST_CONTROL\r\n");
    assert(goals[5] == (3000U | EXCAVATOR_WHEEL_DIRECTION_BIT) && goals[6] == 3000U);
    command("HOST_CONTROL -6 -6 0 0 0 0 0", "OK HOST_CONTROL\r\n");
    assert(goals[5] == 3000U && goals[6] == (3000U | EXCAVATOR_WHEEL_DIRECTION_BIT));
    command("HOST_CONTROL -6 6 0 0 0 0 0", "OK HOST_CONTROL\r\n");
    assert(goals[5] == 3000U && goals[6] == 3000U);
    command("HOST_CONTROL 6 -6 0 0 0 0 0", "OK HOST_CONTROL\r\n");
    assert(goals[5] == (3000U | EXCAVATOR_WHEEL_DIRECTION_BIT) && goals[6] == (3000U | EXCAVATOR_WHEEL_DIRECTION_BIT));
    wireless_timeout_pending = true;
    ServoControl_ProcessWirelessInput();
    assert(g_chassis_armed && !wireless_timeout_pending); /* Radio loss does not kill a valid host lease. */
    ticks += 201;
    ServoControl_UpdateMotion();
    assert(!host_owned && !host_streaming && remote_enabled && !g_chassis_armed);
    assert(goals[4] == 0 && goals[5] == 0 && goals[6] == 0);
    command("HOST_CONTROL 0 0 0 0 0 0 0", "ERR HOST_CONTROL\r\n");
    command("HOST_BEGIN", "OK HOST_BEGIN\r\n");
    command("HOST_CONTROL 7 0 0 0 0 0 0", "ERR HOST_CONTROL\r\n");
    assert(!host_streaming);
    command("HOST_BEGIN", "OK HOST_BEGIN\r\n");
    command("HOST_CONTROL 0 0 0 0 0 0 11", "ERR HOST_CONTROL\r\n");
    command("HOST_BEGIN", "OK HOST_BEGIN\r\n");
    command("HOST_CONTROL 0 0 0 0 0 0 0 extra", "ERR HOST_CONTROL\r\n");
    command("HOST_BEGIN", "OK HOST_BEGIN\r\n");
    ticks += 201; /* Reject late frames even before the motion watchdog is scheduled. */
    command("HOST_CONTROL 0 0 0 0 0 0 0", "ERR HOST_CONTROL\r\n");
    command("HOST_BEGIN", "OK HOST_BEGIN\r\n");
    for (unsigned i = 0; i < 24; i++)
    {
        ticks += 50;
        command("HOST_CONTROL 0 0 0 0 0 0 0", "OK HOST_CONTROL\r\n");
    }
    command("HOST_CONTROL 6 6 0 0 0 0 0", "OK HOST_CONTROL\r\n");
    char flood[257];
    for (unsigned i = 0; i < 32; i++) memcpy(&flood[i * 8], "CHASSIS\n", 8);
    flood[256] = '\0';
    serial_input = flood; serial_index = 0;
    ServoControl_ProcessSerialInput();
    assert(serial_index == SERVO_CONTROL_RX_BUDGET && host_streaming);
    ticks += 201;
    ServoControl_ProcessSerialInput();
    assert(serial_index == 2 * SERVO_CONTROL_RX_BUDGET && !host_streaming && !g_chassis_armed);
    assert(goals[5] == 0 && goals[6] == 0);
    serial_input = NULL; serial_index = 0;
    command("HOST_END", "OK HOST_END\r\n");
    assert(!host_owned && remote_enabled);
    command("RESUME_REMOTE", "OK RESUME_REMOTE\r\n");
    assert(!host_owned && remote_enabled && !remote_armed);
    assert(sync_writes == 0);
}

static void test_radio_host_coexist(void)
{
    const ExcavatorVelocityCmd_t neutral = {0};
    const ExcavatorVelocityCmd_t backward = {.left_track = -6, .right_track = -6};
    const unsigned before_modes = mode_writes;
    command("STOP_CONTROL", "OK STOP_CONTROL\r\n");
    arm_neutral();
    command("HOST_BEGIN", "OK HOST_BEGIN\r\n");
    for (unsigned i = 0; i < 24; i++)
    {
        ticks += 50;
        ServoControl_HandleWireless(&neutral);
        command("HOST_CONTROL 0 0 0 0 0 0 0", "OK HOST_CONTROL\r\n");
    }
    assert(radio_armed && remote_armed && g_chassis_armed);
    const uint32_t raw_count = wireless_input_count;
    command("HOST_CONTROL 6 6 0 0 0 0 0", "OK HOST_CONTROL\r\n");
    assert(wireless_input_count == raw_count); /* Host input must not masquerade as RF telemetry. */
    ticks += 50;
    ServoControl_HandleWireless(&neutral);
    assert(host_owned && goals[5] == 35768U && goals[6] == 3000U);
    /* A radio frame queued concurrently with a UART heartbeat wins in that same call. */
    queued = backward; queue_ready = true; wireless_command_queue = (void *)1;
    command("HOST_CONTROL 6 6 0 0 0 0 0", "ERR HOST_PREEMPTED\r\n");
    assert(!host_owned && !host_streaming && remote_enabled && radio_armed);
    assert(goals[5] == 3000U && goals[6] == 35768U);
    command("HOST_END", "OK HOST_END\r\n");
    command("HOST_BEGIN", "ERR RADIO_ACTIVE\r\n");
    command("HOST_CONTROL 6 6 0 0 0 0 0", "ERR HOST_PREEMPTED\r\n");
    assert(goals[5] == 3000U && goals[6] == 35768U); /* Late host frames/end cannot stop radio. */
    ServoControl_HandleWireless(&neutral);
    assert(goals[5] == 0U && goals[6] == 0U);
    ticks += 50;
    ServoControl_UpdateMotion();
    assert(goals[5] == 0U && goals[6] == 0U && !host_owned); /* No old host action resumes. */
    command("HOST_BEGIN", "OK HOST_BEGIN\r\n");
    for (unsigned i = 0; i < 24; i++)
    {
        ticks += 50;
        command("HOST_CONTROL 0 0 0 0 0 0 0", "OK HOST_CONTROL\r\n");
    }
    command("HOST_CONTROL 6 6 0 0 0 0 0", "OK HOST_CONTROL\r\n");
    wireless_timeout_pending = true;
    ServoControl_ProcessWirelessInput();
    assert(host_owned && goals[5] == 35768U && goals[6] == 3000U && !radio_armed);
    ticks += 201;
    ServoControl_UpdateMotion();
    assert(!host_owned && remote_enabled && goals[5] == 0U && goals[6] == 0U);
    arm_neutral();
    ServoControl_HandleWireless(&backward);
    assert(goals[5] == 3000U && goals[6] == 35768U); /* Host timeout never requires RESUME_REMOTE. */
    command("STOP_CONTROL", "OK STOP_CONTROL\r\n");
    ServoControl_HandleWireless(&backward);
    assert(remote_enabled && !radio_armed && goals[5] == 0U && goals[6] == 0U);
    arm_neutral();
    ServoControl_HandleWireless(&backward);
    assert(goals[5] == 3000U && goals[6] == 35768U);
    command("STOP_CONTROL", "OK STOP_CONTROL\r\n");
    assert(mode_writes == before_modes && sync_writes == 0);
    puts("PASS: radio neutral coexistence, immediate priority, queued RF override, stale UART isolation, independent loss and stop/re-arm");
}

static void cold_servo_boot(void)
{
    const uint16_t references[4] = {1800U, 2400U, 2700U, 1000U};
    const uint8_t ids[4] = {1U, 2U, 3U, 8U};
    for (unsigned i = 0; i < 4; i++)
    {
        feedback[ids[i]] = references[i];
        position_goals[ids[i]] = 0U;
        modes[ids[i]] = ids[i] <= 3U ? 1U : 0U;
        torque[ids[i]] = 0U;
        pending_feedback_shift[ids[i]] = 0;
    }
    for (unsigned id = 4; id <= 6; id++) { modes[id]=0U; torque[id]=0U; goals[id]=0U; }
    fail_read = fail_offset = queue_ready = wireless_timeout_pending = false;
    ServoControl_LoadParameters();
    ServoControl_UpdateFeedback();
    assert(remote_ready_mask == 0U);
}

static void startup_ticks(unsigned samples)
{
    for (unsigned i=0; i<samples; i++)
    {
        ticks += 100;
        ServoControl_UpdateFeedback();
        ServoControl_UpdateStartup();
    }
}

static void test_startup_recovery(void)
{
    cold_servo_boot();
    const unsigned before = position_writes;
    startup_ticks(5);
    assert(position_writes == before && !remote_armed);
    startup_ticks(12);
    assert(strcmp(startup_state,"READY")==0 && remote_ready_mask==23U && !remote_armed && !radio_armed);
    const uint8_t ids[4]={1U,2U,3U,8U};
    const uint16_t expected[4]={1800U,2400U,2700U,1000U};
    for (unsigned i=0;i<4;i++)
    {
        assert(reference_goal(ids[i])==expected[i] && torque[ids[i]]==1U && modes[ids[i]]==0U);
    }
    for (unsigned id=4;id<=6;id++)assert(torque[id]==1U && modes[id]==1U && goals[id]==0U);
    assert(position_writes==before+4U && sync_writes==0U);
    arm_neutral();
    const unsigned retained_writes=position_writes,retained_modes=mode_writes;
    ServoControl_LoadParameters();ServoControl_UpdateFeedback();startup_ticks(15);
    assert(remote_ready_mask==23U && strcmp(startup_state,"READY")==0);
    assert(position_writes==retained_writes && mode_writes==retained_modes); /* MCU-only reset adopts valid holds. */

    cold_servo_boot();
    const ExcavatorVelocityCmd_t deflected={.left_track=6,.right_track=6};
    for (unsigned i=0;i<20;i++)
    {
        ticks+=50;ServoControl_HandleWireless(&deflected);
        ServoControl_UpdateFeedback();ServoControl_UpdateStartup();
    }
    assert(remote_ready_mask==0U && strcmp(startup_state,"WAIT_NEUTRAL")==0);
    const ExcavatorVelocityCmd_t neutral={0};
    for(unsigned i=0;i<50;i++)
    {
        ticks+=50;ServoControl_HandleWireless(&neutral);
        ServoControl_UpdateFeedback();ServoControl_UpdateStartup();
    }
    assert(remote_ready_mask==23U && strcmp(startup_state,"READY")==0);
    assert(remote_armed && radio_armed && g_chassis_armed);
    command("PAUSE_REMOTE","OK PAUSE_REMOTE\r\n");
    const unsigned paused_modes=mode_writes;
    startup_ticks(20);
    assert(!remote_enabled && strcmp(startup_state,"MAINTENANCE")==0 && mode_writes==paused_modes);

    cold_servo_boot();
    modes[1]=1U;torque[1]=1U;
    const unsigned rejected_modes=mode_writes;
    startup_ticks(20);
    assert(strcmp(startup_state,"FAILED")==0 && startup_error==2U && mode_writes==rejected_modes && !remote_enabled);
    assert(torque[1]==1U); /* A wrong-mode loaded axis is not automatically unloaded. */

    cold_servo_boot();
    fail_read=true;startup_ticks(20);
    assert(strcmp(startup_state,"WAIT_SERVO")==0);
    fail_read=false;startup_ticks(20);
    assert(remote_ready_mask==23U && strcmp(startup_state,"READY")==0); /* Servos can power on later. */
    puts("PASS: cold servo power recovery, current-position holds, neutral/stability delay, retained MCU reset, maintenance and loaded-mode refusal");
}

int main(void)
{
    test_boundaries();
    const uint16_t initial[9] = {0, 1269, 2143, 2727, 3227, 2115, 1684, 0, 1292};
    memcpy(feedback, initial, sizeof(initial));
    for (uint8_t id = 1; id < 9; id++) { modes[id] = 1; torque[id] = 1; position_goals[id] = 100; }
    modes[8] = 0;
    for (uint8_t id = 4; id <= 6; id++) { modes[id] = 0; torque[id] = 0; }
    ServoControl_LoadParameters();
    ServoControl_UpdateFeedback();
    ServoControl_UpdateMotion();
    assert(position_writes == 0 && speed_writes == 0 && mode_writes == 0 && sync_writes == 0);
    assert(remote_ready_mask == 0 && !remote_armed);
    for (uint8_t axis = 0; axis < 5; axis++)
        assert(position_axes[axis].limits_enabled == (axis != 3));
    command("FLASH_LIMITS", "FLASH_LIMITS Config2 1001 2950 2223 4076 1547 3609 172 1292\r\n");
    command("MOVE 0 1270 100", "ERR AXIS_LOCKED\r\n");
    command("PREPARE_REMOTE", "OK PREPARE_REMOTE\r\n");
    ServoControl_UpdateFeedback();
    assert(remote_ready_mask == 23 && calibration_ready_mask == 0 && position_writes == 4);
    for (uint8_t axis = 0; axis < 5; axis++)
        if (axis != 3) assert(reference_goal(position_axes[axis].id) == initial[position_axes[axis].id]);
    assert(reference_goal(2) == 2143); /* No jump to lower limit 2223. */
    arm_neutral();
    ServoControl_UpdateMotion();
    assert(position_writes == 4);

    ExcavatorVelocityCmd_t movement = {.boom = 10, .arm = -10, .bucket = -10, .gripper = 10};
    ServoControl_HandleWireless(&movement);
    ServoControl_UpdateMotion();
    assert(reference_goal(1) == 1264 && reference_goal(3) == 2723);
    assert(reference_goal(2) == 2143 && reference_goal(8) == 1292 && position_writes == 6);
    /* Progress through the servo deadband, but bound lead even with stalled feedback. */
    for (unsigned i = 0; i < 50; i++) ServoControl_UpdateMotion();
    assert(reference_goal(1) == 1179 && reference_goal(3) == 2635);
    const unsigned stalled_writes = position_writes;
    ServoControl_UpdateMotion();
    assert(position_writes == stalled_writes);
    ExcavatorVelocityCmd_t neutral = {0};
    ServoControl_HandleWireless(&neutral);
    assert(position_goals[1] == feedback[1] && position_goals[3] == feedback[3] && remote_goal_active_mask == 0);

    movement.boom = 0; movement.arm = 10; movement.bucket = 10; movement.gripper = -10;
    ServoControl_HandleWireless(&movement);
    ServoControl_UpdateMotion();
    assert(reference_goal(2) == 2153 && reference_goal(3) == 2731 && reference_goal(8) == 1288);
    ServoControl_NotifyWirelessTimeout();
    ServoControl_ProcessWirelessInput();
    assert(!remote_armed && !g_chassis_armed && remote_goal_active_mask == 0);
    assert(position_goals[2] == feedback[2] && position_goals[8] == feedback[8]);
    ServoControl_HandleWireless(&movement);
    assert(!remote_armed);
    arm_neutral();
    ServoControl_HandleWireless(&movement);
    ServoControl_UpdateMotion();
    ticks += 201;
    ServoControl_UpdateMotion();
    assert(!remote_armed && !g_chassis_armed && remote_goal_active_mask == 0);
    arm_neutral();
    ServoControl_HandleWireless(&movement);
    ServoControl_UpdateMotion();
    modes[2] = 1;
    feedback[2] += 85;
    ServoControl_UpdateFeedback();
    assert(!remote_armed && (remote_ready_mask & 2) == 0 && torque[2] == 0);

    command("PREPARE_REMOTE", "OK PREPARE_REMOTE\r\n");
    command("CAL_PREPARE 0", "OK CAL_PREPARE\r\n");
    assert(!remote_enabled && !remote_armed);
    command("MOVE 0 1267 101", "ERR SPEED\r\n");
    command("MOVE 0 1001 100", "ERR RANGE\r\n");
    command("MOVE 0 1267 100", "OK MOVE\r\n");
    const unsigned manual_writes = position_writes;
    ServoControl_HandleWireless(&movement);
    ServoControl_UpdateMotion();
    assert(position_writes == manual_writes);
    feedback[1] = 1267 - 85;
    command("STOP 0", "OK STOP\r\n");
    command("LIMIT 0 0 4095", "ERR FLASH_RANGE\r\n");
    command("LIMIT 0 1250 1290", "OK LIMIT\r\n");
    command("CLEAR 0", "OK CLEAR\r\n");
    assert(position_axes[0].limits_enabled && position_axes[0].minimum_position == 1001 && position_axes[0].maximum_position == 2950);
    command("RESUME_REMOTE", "OK RESUME_REMOTE\r\n");
    arm_neutral();
    command("PAUSE_REMOTE", "OK PAUSE_REMOTE\r\n");
    const unsigned paused_writes = position_writes;
    ServoControl_HandleWireless(&movement);
    assert(!remote_enabled && !remote_armed && !g_chassis_armed);
    assert(last_wireless_input.arm == 10 && last_wireless_input.gripper == -10 && position_writes == paused_writes);
    char diagnostic[128];
    snprintf(diagnostic, sizeof(diagnostic), "REMOTE_INPUT 0 0 0 0 10 10 -10 0 %lu\r\n", (unsigned long)wireless_input_count);
    command("REMOTE_INPUT", diagnostic);
    command("POSITION_DEADBAND 1", "POSITION_DEADBAND 2 16 16\r\n");
    assert(position_writes == paused_writes);
    modes[1] = 1;
    feedback[1] += 85;
    pending_feedback_shift[1] = -85;
    feedback_ready_at[1] = ticks + 100000;
    const uint16_t before_mode_switch = feedback[1];
    const TickType_t before_preparation = ticks;
    command("PREPARE_REMOTE", "OK PREPARE_REMOTE\r\n");
    assert(feedback[1] == before_mode_switch - 85 && position_goals[1] == feedback[1]);
    assert(ticks - before_preparation >= 860);
    arm_neutral();
    ServoControl_HandleWireless(&movement);
    ServoControl_UpdateMotion();
    fail_offset = true;
    ServoControl_UpdateFeedback();
    assert(remote_ready_mask == 0 && !remote_armed && !g_chassis_armed && remote_goal_active_mask == 0);
    fail_offset = false;
    assert(sync_writes == 0);
    test_continuous_follow();
    test_loaded_hold_and_lift();
    test_host_control();
    test_radio_host_coexist();
    test_startup_recovery();
    puts("PASS: Config2 Flash limits, no boot/endpoint jumps, all position channels, inward recovery, release/loss stops, mode faults and Qt ownership");
    return 0;
}
