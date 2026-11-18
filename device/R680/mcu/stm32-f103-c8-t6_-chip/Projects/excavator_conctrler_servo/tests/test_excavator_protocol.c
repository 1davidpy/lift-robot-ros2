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

    ExcavatorProtocol_BuildVelocityFromAdc(&adc, &command);
    assert(command.swing == 0);
    assert(command.boom == 0);
    assert(command.arm == 0);
    assert(command.gripper == 0);

    adc.left_x = 179U;
    ExcavatorProtocol_BuildVelocityFromAdc(&adc, &command);
    assert(command.swing == EXCAVATOR_JOYSTICK_LEVEL_MAX);
    adc.left_x = 3844U;
    ExcavatorProtocol_BuildVelocityFromAdc(&adc, &command);
    assert(command.swing == -EXCAVATOR_JOYSTICK_LEVEL_MAX);
    adc.left_x = 1967U;

    adc.left_y = 81U;
    ExcavatorProtocol_BuildVelocityFromAdc(&adc, &command);
    assert(command.arm == EXCAVATOR_JOYSTICK_LEVEL_MAX);
    adc.left_y = 4008U;
    ExcavatorProtocol_BuildVelocityFromAdc(&adc, &command);
    assert(command.arm == -EXCAVATOR_JOYSTICK_LEVEL_MAX);
    adc.left_y = 1967U;

    adc.right_y = 246U;
    ExcavatorProtocol_BuildVelocityFromAdc(&adc, &command);
    assert(command.boom == -EXCAVATOR_JOYSTICK_LEVEL_MAX);
    adc.right_y = 4052U;
    ExcavatorProtocol_BuildVelocityFromAdc(&adc, &command);
    assert(command.boom == EXCAVATOR_JOYSTICK_LEVEL_MAX);
    adc.right_y = 2086U;

    adc.right_x = 201U;
    ExcavatorProtocol_BuildVelocityFromAdc(&adc, &command);
    assert(command.gripper == -EXCAVATOR_JOYSTICK_LEVEL_MAX);
    adc.right_x = 3884U;
    ExcavatorProtocol_BuildVelocityFromAdc(&adc, &command);
    assert(command.gripper == EXCAVATOR_JOYSTICK_LEVEL_MAX);
}

static void TestSimultaneousJoystickAxes(void)
{
    ExcavatorControlAdc_t adc = {
        .left_y = 500U,
        .left_x = 1967U,
        .right_y = 3000U,
        .right_x = 1975U,
    };
    ExcavatorVelocityCmd_t command;

    ExcavatorProtocol_BuildVelocityFromAdc(&adc, &command);
    assert(command.swing == 0);
    assert(command.boom == EXCAVATOR_JOYSTICK_LEVEL_MAX);
    assert(command.arm == EXCAVATOR_JOYSTICK_LEVEL_MAX);
    assert(command.gripper == 0);

    adc.right_y = 3900U;
    ExcavatorProtocol_BuildVelocityFromAdc(&adc, &command);
    assert(command.boom == EXCAVATOR_JOYSTICK_LEVEL_MAX);
    assert(command.arm == EXCAVATOR_JOYSTICK_LEVEL_MAX);

    adc.left_y = 1967U;
    ExcavatorProtocol_BuildVelocityFromAdc(&adc, &command);
    assert(command.swing == 0);
    assert(command.boom == EXCAVATOR_JOYSTICK_LEVEL_MAX);
    assert(command.arm == 0);
    assert(command.gripper == 0);

    adc.right_y = 2086U;
    ExcavatorProtocol_BuildVelocityFromAdc(&adc, &command);
    adc.left_x = 179U;
    adc.right_x = 3884U;
    ExcavatorProtocol_BuildVelocityFromAdc(&adc, &command);
    assert(command.swing == EXCAVATOR_JOYSTICK_LEVEL_MAX);
    assert(command.boom == 0);
    assert(command.arm == 0);
    assert(command.gripper == EXCAVATOR_JOYSTICK_LEVEL_MAX);
}

static void TestOneAxisPerJoystick(void)
{
    ExcavatorControlAdc_t adc = {
        .left_y = 500U,
        .left_x = 1700U,
        .right_y = 1800U,
        .right_x = 3884U,
    };
    ExcavatorVelocityCmd_t command;

    ExcavatorProtocol_BuildVelocityFromAdc(&adc, &command);
    assert(command.swing == 0);
    assert(command.arm == EXCAVATOR_JOYSTICK_LEVEL_MAX);
    assert(command.boom == 0);
    assert(command.gripper == EXCAVATOR_JOYSTICK_LEVEL_MAX);

    adc.left_x = 179U;
    adc.right_y = 4052U;
    ExcavatorProtocol_BuildVelocityFromAdc(&adc, &command);
    assert(command.swing == EXCAVATOR_JOYSTICK_LEVEL_MAX);
    assert(command.arm == 0);
    assert(command.boom == EXCAVATOR_JOYSTICK_LEVEL_MAX);
    assert(command.gripper == 0);
}

static void AssertTracks(uint8_t mask, uint8_t speed_level, int8_t left, int8_t right)
{
    ExcavatorVelocityCmd_t command = {0};

    ExcavatorProtocol_ApplyDriveButtons(mask, speed_level, &command);
    assert(command.left_track == left);
    assert(command.right_track == right);
}

static void TestDriveButtons(void)
{
    AssertTracks(0U, 1U, 0, 0);
    AssertTracks(BUTTON_L1, 2U, -3, 3);
    AssertTracks(BUTTON_R1, 3U, 3, -3);
    AssertTracks(BUTTON_L1 | BUTTON_R1, 4U, 3, 3);
    AssertTracks(BUTTON_L3, 5U, 0, 0);
    AssertTracks(BUTTON_R3, 6U, 0, 0);
    AssertTracks(BUTTON_L3 | BUTTON_R3, 6U, -3, -3);
    AssertTracks(BUTTON_L1 | BUTTON_L3, 6U, -3, 3);
    AssertTracks(BUTTON_L1, 0U, -3, 3);
    AssertTracks(BUTTON_L1, 7U, -3, 3);
    ExcavatorVelocityCmd_t command = {0};
    ExcavatorProtocol_ApplyDriveButtons(BUTTON_L3, 3, &command);
    assert(command.bucket == -10 && command.left_track == 0 && command.right_track == 0);
    ExcavatorProtocol_ApplyDriveButtons(BUTTON_R3, 3, &command);
    assert(command.bucket == 10 && command.left_track == 0 && command.right_track == 0);
    ExcavatorProtocol_ApplyDriveButtons(BUTTON_L3 | BUTTON_R3, 3, &command);
    assert(command.bucket == 0 && command.left_track == -3 && command.right_track == -3);
    for (uint8_t mask = 0; mask < 16; mask++)
    {
        ExcavatorProtocol_ApplyDriveButtons(mask, 1, &command);
        if ((mask & (BUTTON_L3 | BUTTON_R3)) == (BUTTON_L3 | BUTTON_R3))
            assert(command.bucket == 0 && command.left_track == -3 && command.right_track == -3);
    }
}

static void TestServoReceiverPacketIdentity(void)
{
    const ExcavatorVelocityCmd_t command = {
        .left_track = 6,
        .right_track = 6,
        .swing = 10,
        .bucket = -10,
        .gripper = 10,
    };
    ControlPacket_t packet;

    ExcavatorProtocol_BuildControlPacket(&packet, 42U, &command);
    assert(packet.device_id == 0x02U);
    assert(packet.seq == 42U);
    assert(packet.left_track == command.left_track);
    assert(packet.right_track == command.right_track);
    assert(packet.swing == command.swing);
    assert(packet.bucket == -10 && packet.gripper == 10 && sizeof(packet) == 15);
    assert(ExcavatorProtocol_ValidateControlPacket(&packet));
    packet.version = 1;
    assert(!ExcavatorProtocol_ValidateControlPacket(&packet));
    packet.version = 2; packet.gripper = 0;
    assert(!ExcavatorProtocol_ValidateControlPacket(&packet));
}

int main(void)
{
    if (EXCAVATOR_CHASSIS_TEST_MODE)
    {
        ExcavatorControlAdc_t adc = {.left_x = 1967U};
        ExcavatorVelocityCmd_t command;
        ExcavatorProtocol_BuildVelocityFromAdc(&adc, &command);
        assert(command.swing == 0 && command.boom == 0 && command.arm == 0 && command.bucket == 0);
        adc.left_x = 0U;
        ExcavatorProtocol_BuildVelocityFromAdc(&adc, &command);
        assert(command.swing == 10 && command.boom == 0 && command.arm == 0 && command.bucket == 0);
        adc.left_x = 4095U;
        adc.left_y = 0U;
        adc.right_x = 4095U;
        adc.right_y = 4095U;
        ExcavatorProtocol_BuildVelocityFromAdc(&adc, &command);
        assert(command.swing == -10 && command.boom == 0 && command.arm == 0 && command.bucket == 0);
    }
    else
    {
        TestJoystickDirections();
        TestSimultaneousJoystickAxes();
        TestOneAxisPerJoystick();
    }
    TestDriveButtons();
    TestServoReceiverPacketIdentity();
    puts("excavator protocol mapping tests passed");
    return 0;
}
