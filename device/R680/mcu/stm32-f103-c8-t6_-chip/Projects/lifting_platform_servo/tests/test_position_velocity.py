"""Check production monitor velocity against raw counts and real sample ticks."""
import os
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[3]
src = (root / 'Projects/lifting_platform_servo/App/Tasks/servo_jog_task.c').read_text()
start = src.index('static bool ReadMonitors(void)')
body = src[start:src.index('/* P0B-77/79', start)]
stub = r'''
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <stdio.h>
typedef uint32_t TickType_t;
typedef int BspIsl60prResult;
#define BSP_ISL60PR_OK 0
#define pdMS_TO_TICKS(x) (x)
#define portTICK_PERIOD_MS 1
#define SERVO_COMMAND_UNITS_PER_REVOLUTION 10000U
#define SERVO_MONITOR_SPEED_REGISTER 1
#define SERVO_MONITOR_STATUS_REGISTER 2
#define SERVO_MONITOR_FAULT_REGISTER 3
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
static struct { int32_t count,origin,up_sign; bool valid,monitor_initialized; } lift;
static struct {
 int32_t encoder_count,speed_rpm,position_speed_rpm,height_um;
 uint32_t position_sample_tick,status_word,fault_code,position_valid,position_speed_valid;
} g_servo_jog_diagnostics;
static uint32_t tick, fake_position;
static bool fail_read;
static TickType_t xTaskGetTickCount(void) {return tick;}
static int BSP_ISL60PR_ReadRegister32(int reg,uint32_t *v) {
 (void)reg;*v=fake_position;tick+=40;return fail_read?1:0;
}
static int BSP_ISL60PR_ReadRegister(int reg,uint16_t *v) {
 *v=reg==1?999:0;tick+=40;return fail_read?1:0;
}
static void UpdateStopObservation(void) {}
static void ServoJog_RecordResult(int value) {(void)value;}
'''
main = r'''
int main(void) {
 lift.valid=true;lift.up_sign=1;tick=1000;fake_position=0;
 assert(ReadMonitors());assert(!g_servo_jog_diagnostics.position_speed_valid);
 tick=1200;fake_position=10000;
 assert(ReadMonitors());assert(g_servo_jog_diagnostics.position_speed_rpm==300);
 assert(g_servo_jog_diagnostics.speed_rpm==999);
 assert(g_servo_jog_diagnostics.position_sample_tick==1200);
 tick=1400;fake_position=0;assert(ReadMonitors());
 assert(g_servo_jog_diagnostics.position_speed_rpm==-300);
 tick=1600;assert(ReadMonitors());assert(g_servo_jog_diagnostics.position_speed_rpm==0);
 assert(g_servo_jog_diagnostics.speed_rpm==999); /* filtered speed may lag standstill */
 tick=1600;assert(ReadMonitors());assert(!g_servo_jog_diagnostics.position_speed_valid);
 tick=2200;assert(ReadMonitors());assert(!g_servo_jog_diagnostics.position_speed_valid);
 tick=2400;fail_read=true;assert(!ReadMonitors());
 assert(g_servo_jog_diagnostics.position_sample_tick==2200);
 fail_read=false;tick=0xfffffff0U;fake_position=0;assert(ReadMonitors());
 tick=184;fake_position=10000;assert(ReadMonitors());
 assert(g_servo_jog_diagnostics.position_speed_valid && g_servo_jog_diagnostics.position_speed_rpm==300);
 tick=185;fake_position=0x80000000U;assert(ReadMonitors());
 assert(!g_servo_jog_diagnostics.position_speed_valid);
 puts("PASS: signed count velocity, filtered-speed independence, timestamp, stale/zero interval, failure, wrap, overflow");
 return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='lift-count-speed-') as directory:
    path = Path(directory)
    (path / 'test.c').write_text(stub + body + main)
    subprocess.run([os.environ.get('TCC', 'tcc'), str(path / 'test.c'), '-o', str(path / 'test.exe')], check=True)
    subprocess.run([str(path / 'test.exe')], check=True)
