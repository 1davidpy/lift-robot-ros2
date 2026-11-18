#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "excavator_protocol.h"

#define BUTTON_L1  (1U << 0U)
#define BUTTON_R1  (1U << 1U)
#define BUTTON_L3  (1U << 2U)
#define BUTTON_R3  (1U << 3U)

static void TestJoystickDirections(void)
{
    ExcavatorControlAdc_t adc = {
        .left_y = 1967U,
        .left_x = 1967U,
        .right_y = 2086U,
        .right_x = 1975U,
    };
    ExcavatorVelocityCmd_t command;
    uint8_t active_axis = EXCAVATOR_ACTUATOR_AXIS_NONE;

    ExcavatorProtocol_BuildVelocityFromAdc(&adc, &active_axis, &command);
    assert(command.swing == 0);
    assert(command.boom == 0);
    assert(command.arm == 0);
    assert(command.bucket == 0);

    adc.left_x = 179U;
    ExcavatorProtocol_BuildVelocityFromAdc(&adc, &active_axis, &command);
    assert(command.swing == EXCAVATOR_JOYSTICK_LEVEL_MAX);
    adc.left_x = 3844U;
    ExcavatorProtocol_BuildVelocityFromAdc(&adc, &active_axis, &command);
    assert(command.swing == -EXCAVATOR_JOYSTICK_LEVEL_MAX);
    adc.left_x = 1967U;

    adc.left_y = 81U;
    ExcavatorProtocol_BuildVelocityFromAdc(&adc, &active_axis, &command);
    assert(command.arm == EXCAVATOR_JOYSTICK_LEVEL_MAX);
    adc.left_y = 4008U;
    ExcavatorProtocol_BuildVelocityFromAdc(&adc, &active_axis, &command);
    assert(command.arm == -EXCAVATOR_JOYSTICK_LEVEL_MAX);
    adc.left_y = 1967U;

    adc.right_y = 246U;
    ExcavatorProtocol_BuildVelocityFromAdc(&adc, &active_axis, &command);
    assert(command.boom == -EXCAVATOR_JOYSTICK_LEVEL_MAX);
    adc.right_y = 4052U;
    ExcavatorProtocol_BuildVelocityFromAdc(&adc, &active_axis, &command);
    assert(command.boom == EXCAVATOR_JOYSTICK_LEVEL_MAX);
    adc.right_y = 2086U;

    adc.right_x = 201U;
    ExcavatorProtocol_BuildVelocityFromAdc(&adc, &active_axis, &command);
    assert(command.bucket == -EXCAVATOR_JOYSTICK_LEVEL_MAX);
    adc.right_x = 3884U;
    ExcavatorProtocol_BuildVelocityFromAdc(&adc, &active_axis, &command);
    assert(command.bucket == EXCAVATOR_JOYSTICK_LEVEL_MAX);
}

static void TestLargestJoystickAxisWins(void)
{
    ExcavatorControlAdc_t adc = {
        .left_y = 500U,
        .left_x = 1967U,
        .right_y = 3000U,
        .right_x = 1975U,
    };
    ExcavatorVelocityCmd_t command;
    uint8_t active_axis = EXCAVATOR_ACTUATOR_AXIS_NONE;

    ExcavatorProtocol_BuildVelocityFromAdc(&adc, &active_axis, &command);
    assert(command.swing == 0);
    assert(command.boom == 0);
    assert(command.arm == EXCAVATOR_JOYSTICK_LEVEL_MAX);
    assert(command.bucket == 0);

    adc.right_y = 3900U;
    ExcavatorProtocol_BuildVelocityFromAdc(&adc, &active_axis, &command);
    assert(command.boom == 0);
    assert(command.arm == EXCAVATOR_JOYSTICK_LEVEL_MAX);

    adc.left_y = 1967U;
    ExcavatorProtocol_BuildVelocityFromAdc(&adc, &active_axis, &command);
    assert(command.swing == 0);
    assert(command.boom == EXCAVATOR_JOYSTICK_LEVEL_MAX);
    assert(command.arm == 0);
    assert(command.bucket == 0);

    adc.right_y = 2086U;
    ExcavatorProtocol_BuildVelocityFromAdc(&adc, &active_axis, &command);
    adc.left_x = 179U;
    adc.right_x = 3884U;
    ExcavatorProtocol_BuildVelocityFromAdc(&adc, &active_axis, &command);
    assert(command.swing == 0);
    assert(command.boom == 0);
    assert(command.arm == 0);
    assert(command.bucket == EXCAVATOR_JOYSTICK_LEVEL_MAX);
}

static void AssertTracks(uint8_t mask, int8_t left, int8_t right)
{
    ExcavatorVelocityCmd_t command = {0};

    ExcavatorProtocol_ApplyDriveButtons(mask, &command);
    assert(command.left_track == left);
    assert(command.right_track == right);
}

static void TestDriveButtons(void)
{
    AssertTracks(0U, 0, 0);
    AssertTracks(BUTTON_L1,
                 EXCAVATOR_TRACK_LEVEL_MAX,
                 -EXCAVATOR_TRACK_LEVEL_MAX);
    AssertTracks(BUTTON_R1,
                 -EXCAVATOR_TRACK_LEVEL_MAX,
                 EXCAVATOR_TRACK_LEVEL_MAX);
    AssertTracks(BUTTON_L1 | BUTTON_R1,
                 EXCAVATOR_TRACK_LEVEL_MAX,
                 EXCAVATOR_TRACK_LEVEL_MAX);
    AssertTracks(BUTTON_L3 | BUTTON_R3,
                 -EXCAVATOR_TRACK_LEVEL_MAX,
                 -EXCAVATOR_TRACK_LEVEL_MAX);
    AssertTracks(BUTTON_L3, 0, 0);
    AssertTracks(BUTTON_R3, 0, 0);
    AssertTracks(BUTTON_L1 | BUTTON_L3, 0, 0);
}

int main(void)
{
    TestJoystickDirections();
    TestLargestJoystickAxisWins();
    TestDriveButtons();
    puts("excavator protocol mapping tests passed");
    return 0;
}
