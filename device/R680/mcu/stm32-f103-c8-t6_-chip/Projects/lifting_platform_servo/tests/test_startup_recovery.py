"""Run the production reconnect/reference decision and reset gate against stubs."""
import os
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[3]
source = (root / 'Projects/lifting_platform_servo/App/Tasks/servo_jog_task.c').read_text()
start = source.index('        if (lift.reference_pending && Stationary()')
boot = source[start:source.index('        UpdateDirectionalLimits();', start)]
start = source.index('bool ServoJog_RequestReset(void)')
reset = source[start:source.index('static bool LimitBlocks(', start)]
stub = r'''
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(x) (x)
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define BSP_ISL60PR_JOG_STOP 0
#define SERVO_COMMAND_UNITS_PER_REVOLUTION 10000U
#define LIFT_PROFILE_BOOT_HOME_ARMED 8U
#define RESULT_NEED_REFERENCE 2U
#define RESULT_OK 0U
#define SERVO_TARGET_STATUS_IDLE 0U
#define SERVO_TARGET_STATUS_REJECTED_NO_POSITION 0xE1U
static struct {
 bool reference_pending,boot_home_checked,valid,absolute_return,route_active,configured;
 uint32_t last_reference_attempt,moving;
 struct {uint32_t profile_flags,absolute_stage;} saved;
} lift;
static struct {
 uint32_t command_result,target_position,target_height_valid,target_status,command_units_per_rev;
 uint32_t brake_released,commanded_speed,reset_pending,ready_for_command;
} g_servo_jog_diagnostics;
static uint32_t tick, processing_source, pending_command,reset_requested_at, moves, restores;
static bool stopped=true, restore_ok=true, reset_pending,stop_pending,remote_owned;
static TickType_t xTaskGetTickCount(void) {return tick;}
static bool Stationary(void) {return stopped;}
static void ServoJog_SetBrakeReleased(bool value) {g_servo_jog_diagnostics.brake_released=value;}
static bool RestoreAbsoluteReference(void) {restores++;lift.valid=restore_ok;return restore_ok;}
static bool MoveTo(int target) {assert(target==0);moves++;lift.moving=1;return true;}
static void setup(void) {
 memset(&lift,0,sizeof(lift));memset(&g_servo_jog_diagnostics,0,sizeof(g_servo_jog_diagnostics));
 lift.reference_pending=true;lift.saved.absolute_stage=2;lift.saved.profile_flags=8;
 g_servo_jog_diagnostics.command_units_per_rev=10000;
 tick=1000;processing_source=pending_command=moves=restores=0;
 reset_pending=stop_pending=remote_owned=false;stopped=restore_ok=true;
}
'''
main = r'''
int main(void) {
 setup();restore_ok=false;boot_step(false,2);assert(!lift.boot_home_checked && !moves);
 tick+=500;restore_ok=true;boot_step(false,2);assert(moves==1 && lift.valid && lift.boot_home_checked);
 tick+=500;boot_step(false,2);assert(moves==1 && processing_source==2);
 setup();lift.boot_home_checked=true;boot_step(false,0);assert(lift.valid && !moves && !lift.reference_pending);
 setup();boot_step(true,0);assert(!restores && lift.boot_home_checked);
 tick+=500;boot_step(false,0);assert(lift.valid && !moves);
 setup();lift.saved.absolute_stage=1;restore_ok=false;boot_step(false,0);
 assert(!lift.reference_pending && !moves && g_servo_jog_diagnostics.target_status==0xE1);
 setup();lift.saved.profile_flags=0;boot_step(false,0);assert(lift.valid && !moves);
 setup();lift.reference_pending=false;boot_step(false,0);assert(!restores && !moves);
 setup();lift.configured=true;assert(ServoJog_RequestReset());assert(reset_pending && reset_requested_at==tick);
 assert(!ServoJog_RequestReset());
 setup();lift.route_active=true;assert(!ServoJog_RequestReset());
 setup();g_servo_jog_diagnostics.brake_released=1;assert(!ServoJog_RequestReset());
 setup();lift.configured=true;stopped=false;assert(!ServoJog_RequestReset());
 setup();pending_command=1;assert(!ServoJog_RequestReset());
 setup();remote_owned=true;assert(!ServoJog_RequestReset());
 setup();stop_pending=true;assert(!ServoJog_RequestReset());
 setup();lift.configured=false;stopped=false;assert(ServoJog_RequestReset());
 puts("PASS: delayed/transient startup, one-shot HOME, reconnect without replay, key/STOP suppression, unpaired reference, reset gates");
 return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='lift-startup-') as directory:
    path = Path(directory)
    (path / 'test.c').write_text(stub + 'static void boot_step(bool keys, uint8_t command_source) {\n' + boot + '}\n' + reset + main)
    subprocess.run([os.environ.get('TCC', 'tcc'), str(path / 'test.c'), '-o', str(path / 'test.exe')], check=True)
    subprocess.run([str(path / 'test.exe')], check=True)
