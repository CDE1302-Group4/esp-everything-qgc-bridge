"""Execute the actual mission-task body with mocked RTOS/PX4 feedback.

TAKEOFF_SOURCE_DIR selects staged sources before applying the firmware patch.
This checks command sequencing; it is not an ESP-IDF build or physical flight.
"""
import os
from pathlib import Path
import re
import shutil
import subprocess

import pytest


def test_firmware_takeoff_sequence(tmp_path):
    compiler = shutil.which('gcc')
    if compiler is None:
        pytest.skip('gcc required')
    source = Path(os.environ.get('TAKEOFF_SOURCE_DIR', Path(__file__).resolve().parents[2] / 'main'))
    # Compile the complete mission task, excluding hardware-only app_main startup.
    body = (source / 'main.c').read_text().split('void app_main(void)')[0]
    body = re.sub(r'^#include .*$', '', body, flags=re.MULTILINE)
    harness = r'''
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <setjmp.h>
#include "takeoff_profile.h"
#define TOF_ENABLED 0
#define PX4_MAIN_MODE_OFFBOARD 6
#define pdMS_TO_TICKS(x) (x)
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGE(...) ((void)0)
typedef struct { float x,y,z,vz,heading; bool armed; int custom_main_mode; uint32_t last_hb_ms; } drone_state_t;
enum {MISSION_BUSY, MISSION_READY, MISSION_FLYING};
static drone_state_t state = {.x=1, .y=2, .z=-0.2f, .last_hb_ms=1};
static int64_t clock_us=1000000, arm_time;
static int scenario, ready_count, land_count, phase, hold_count;
static bool reached_flying;
static float commanded_z=-0.2f, held_x, reference_z;
static jmp_buf finished;
static int64_t esp_timer_get_time(void) { return clock_us; }
static drone_state_t mavlink_get_state(void) { return state; }
static bool mavlink_position_valid(void) {
    return !(scenario==2 && state.armed && clock_us-arm_time>2000000);
}
static void vTaskDelay(int ms) {
    clock_us += ms*1000;
    assert(clock_us<40000000);
    if (!state.armed && ready_count==1 && clock_us<4000000) state.x += 0.02f;
    if (state.armed && scenario!=1) {
        state.vz = (commanded_z-state.z)/(ms/1000.0f);
        state.z = commanded_z;
    }
    if (scenario==3 && state.armed && clock_us-arm_time>2000000) state.custom_main_mode=3;
    if (scenario==3 && clock_us-arm_time>4000000) state.armed=false;
}
static void mavlink_set_hold(void) { held_x=state.x; commanded_z=state.z; hold_count++; }
static void nav_cancel(void) {}
static void wifi_clear_start_request(void) {}
static void wifi_clear_land_request(void) {}
static void wifi_set_ground_z(float z) { reference_z=z; }
static void wifi_set_mission_phase(int p) {
    phase=p;
    if (p==MISSION_READY && ++ready_count==2) longjmp(finished,1);
    if (p==MISSION_FLYING) {
        assert(scenario==0);
        assert(fabsf(state.z-(-0.7f))<0.001f);
        assert(fabsf(state.vz)<0.1f);
        assert(clock_us-arm_time>=6000000);
        reached_flying=true;
    }
}
static bool wifi_start_requested(void) { return clock_us>=4000000; }
static bool wifi_land_requested(void) { return phase==MISSION_FLYING; }
static void mavlink_set_offboard_mode(void) { state.custom_main_mode=6; }
static void mavlink_arm(bool armed) {
    if (armed) {
        assert(fabsf(held_x-state.x)<0.0001f);
        assert(fabsf(reference_z+0.2f)<0.0001f);
        arm_time=clock_us;
    }
    state.armed=armed;
}
static void mavlink_send_land_command(void) { land_count++; state.armed=false; }
static void mavlink_set_position_ned(float x,float y,float z,float yaw) {
    assert(isnan(yaw));
    assert(fabsf(x-held_x)<0.0001f && y==2);
    assert(z>=-0.70001f && z<=-0.19999f);
    assert(fabsf(z-commanded_z)<=0.00751f);  /* max climb speed 0.15 m/s */
    commanded_z=z;
}
'''
    tail = r'''
int main(int argc, char **argv) {
    scenario=atoi(argv[1]);
    assert(fabsf(takeoff_target_z(-0.2f,0)+0.2f)<0.0001f);
    assert(fabsf(takeoff_target_z(-0.2f,5)+0.7f)<0.0001f);
    assert(!takeoff_at_hover(-0.5f,0,-0.2f));
    assert(!takeoff_at_hover(-0.7f,0.2f,-0.2f));
    if (!setjmp(finished)) mission_task(NULL);
    assert(hold_count>2);
    assert(reached_flying==(scenario==0));
    assert(land_count==(scenario==3 ? 0 : 1));
    return 0;
}
'''
    path = tmp_path / 'takeoff.c'
    path.write_text(harness + body + tail)
    exe = tmp_path / 'takeoff'
    subprocess.run([compiler, '-std=c11', '-I', str(source), str(path), '-lm', '-o', str(exe)], check=True)
    for scenario in range(4):
        subprocess.run([str(exe), str(scenario)], check=True, timeout=5)


def test_firmware_telemetry_layout_matches_python(tmp_path):
    compiler = shutil.which('gcc')
    if compiler is None:
        pytest.skip('gcc required')
    main_dir = Path(__file__).resolve().parents[2] / 'main'
    source = Path(os.environ.get('TAKEOFF_SOURCE_DIR', main_dir))
    (tmp_path / 'driver').mkdir()
    (tmp_path / 'driver/i2c_master.h').write_text('#pragma once\n')
    harness = tmp_path / 'wire.c'
    harness.write_text(r'''
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "wifi_task.h"
_Static_assert(sizeof(wifi_telem_pkt_t)==90, "wire size");
_Static_assert(offsetof(wifi_telem_pkt_t,flight_magic)==55, "legacy prefix");
int main(void) {
    wifi_telem_pkt_t p={.pkt_type=1,.drone_id=15,.local_z=-0.7f,.local_vz=0.02f,
        .ground_z=-0.2f,.armed=1,.main_mode=6,.mission_phase=2,.position_age_ms=20,
        .heartbeat_age_ms=150,.map_offset_x=-2,.map_offset_y=3};
    memcpy(p.flight_magic,"TF01",4);
    return fwrite(&p,1,sizeof(p),stdout)==sizeof(p) ? 0 : 1;
}
''')
    exe = tmp_path / 'wire'
    subprocess.run([compiler, '-std=c11', '-I', str(source), '-I', str(main_dir), '-I', str(tmp_path),
                    str(harness), '-o', str(exe)], check=True)
    wire = subprocess.check_output([str(exe)])
    import sys
    sys.path.insert(0, str(main_dir.parent / 'laptop'))
    from protocol import parse_telemetry
    p = parse_telemetry(wire)
    assert p.flight_status_version == 1 and p.px4_link_ok
    assert p.ground_z-p.local_z == pytest.approx(0.5)
    assert p.map_offset_x == -2 and p.map_offset_y == 3
