"""Exercise the production StartMotion function with timed drive/relay stubs."""
import os
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[3]
source = (root / 'Projects/lifting_platform_servo/App/Tasks/servo_jog_task.c').read_text()
start = source.index('static bool StartMotion(')
function = source[start:source.index('static bool StartRouteSegment(', start)]
stub = r'''
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
typedef uint32_t TickType_t;
typedef int BspIsl60prResult;
typedef int BspIsl60prJogCommand;
#define BSP_ISL60PR_OK 0
#define pdMS_TO_TICKS(x) (x)
#define SERVO_BRAKE_ZERO_CONFIRM_MS 200U
#define SERVO_BRAKE_RELEASE_SETTLE_TIME_MS 300U
#define SERVO_BRAKE_STABLE_POSITION_UNITS 20L
#define MONITOR_PERIOD_MS 50U
#define SERVO_MANUAL_SPEED_PULSES_PER_SECOND 500000U
#define PP_ACCEL 500000U
#define SERVO_JOG_DECELERATION_PULSES_PER_SECOND2 1000000U
#define SERVO_PP_DECELERATION_PULSES_PER_SECOND2 200000U
#define BSP_KEY_TOP_LEFT 0
#define BSP_KEY_TOP_RIGHT 1
#define BSP_ISL60PR_JOG_STOP 0
#define SERVO_JOG_STATE_CLOCKWISE 4
#define SERVO_JOG_STATE_COUNTERCLOCKWISE 5
enum {RESULT_CANCELED=9,RESULT_PROFILE=11,RESULT_NEED_REFERENCE=2,RESULT_LIMIT=3,RESULT_BUSY=1};
static struct {
 bool valid, home, home_fine, manual_control, upper_return_seen, lower_return_seen;
 bool zero_confirming, stop_tracking, arrival_confirming;
 int32_t count, target;
 unsigned mode, moving, stopping_direction, started_at, profile_speed;
} lift;
static struct {
 unsigned motion_inhibited, command_result, clockwise_command, brake_released;
 unsigned guard_reason, active_command, commanded_speed, motion_start_count, state;
 int guard_speed_rpm, speed_rpm, guard_error_units;
} g_servo_jog_diagnostics;
static unsigned processing_source;
static bool remote_cancel_requested, stop_pending;
static unsigned tick, release_tick, sample_tick, moves, reads_released, scenario, stops;
static TickType_t xTaskGetTickCount(void) { return tick; }
static void vTaskDelay(unsigned ms) { tick += ms; }
static bool LimitBlocks(uint16_t direction) { (void)direction; return false; }
static bool BSP_Key_IsPressed(int key) { (void)key; return false; }
static void ServoJog_SetBrakeReleased(bool value) {
 g_servo_jog_diagnostics.brake_released=value;
 if(value)release_tick=tick;
}
static int BSP_ISL60PR_WriteRegister(unsigned reg,unsigned value) {
 (void)reg;(void)value;tick+=40;return 0;
}
static int BSP_ISL60PR_WriteRegister32(unsigned reg,unsigned value) {
 return BSP_ISL60PR_WriteRegister(reg,value);
}
static int BSP_ISL60PR_SetJogSpeed(unsigned value) { return BSP_ISL60PR_WriteRegister(0,value); }
static int BSP_ISL60PR_Jog(int direction) { (void)direction;assert(0);return 0; }
static int BSP_ISL60PR_MoveAbsolute(int32_t target) {
 (void)target;assert(g_servo_jog_diagnostics.brake_released);
 assert(tick-release_tick>=300);assert(tick-sample_tick<=160);
 assert(reads_released>=2);moves++;return 0;
}
static bool ReadMonitors(void) {
 sample_tick=tick;tick+=160;
 if(g_servo_jog_diagnostics.brake_released) {
  reads_released++;
  if(scenario==1)lift.count+=21;
  if(scenario==2)return false;
  if(scenario==3)stop_pending=true;
  if(scenario==4)remote_cancel_requested=true;
  if(scenario==5)g_servo_jog_diagnostics.speed_rpm=20;
 }
 return true;
}
static bool StoppedSpeedFeedback(void) { return g_servo_jog_diagnostics.speed_rpm==0; }
static void ServoJog_RecordResult(int r) { (void)r; }
static void CommunicationFault(void) { ServoJog_SetBrakeReleased(false);lift.valid=false; }
static bool StopMotion(bool emergency) { (void)emergency;stops++;return true; }
'''
main = r'''
int main(void) {
 for(scenario=0;scenario<6;scenario++) {
  memset(&lift,0,sizeof(lift));memset(&g_servo_jog_diagnostics,0,sizeof(g_servo_jog_diagnostics));
  tick=release_tick=sample_tick=moves=reads_released=stops=0;
  processing_source=2;remote_cancel_requested=stop_pending=false;
  lift.valid=true;g_servo_jog_diagnostics.clockwise_command=1;
  bool ok=StartMotion(1,true,200000,500000);
  assert(ok==(scenario==0));assert(moves==(scenario==0));
  if(scenario==1 || scenario==5)assert(stops && g_servo_jog_diagnostics.motion_inhibited);
  if(scenario==2)assert(!lift.valid && !g_servo_jog_diagnostics.brake_released);
  if(scenario==3 || scenario==4)assert(stops && !g_servo_jog_diagnostics.brake_released);
 }
 puts("PASS: timed brake feedback, drift/speed rejection, read failure, STOP and remote cancel");
 return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='lift-brake-release-') as directory:
    path = Path(directory)
    (path / 'test.c').write_text(stub + function + main)
    subprocess.run([os.environ.get('TCC', 'tcc'), str(path / 'test.c'), '-o', str(path / 'test.exe')], check=True)
    subprocess.run([str(path / 'test.exe')], check=True)
