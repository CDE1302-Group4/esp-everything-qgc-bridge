"""Compile the actual navigator on the host with mocked hardware/RTOS APIs.

NAV_SOURCE_DIR can point to staged firmware sources before applying a patch.
This validates setpoints, not PX4's physical response or an ESP-IDF build.
"""

import os
from pathlib import Path
import shutil
import subprocess

import pytest


def test_waypoints_keep_heading_including_arrival_and_return(tmp_path):
    compiler = shutil.which("gcc")
    if compiler is None:
        pytest.skip("gcc is needed for the navigator host test")
    main_dir = Path(__file__).resolve().parents[2] / "main"
    source_dir = Path(os.environ.get("NAV_SOURCE_DIR", main_dir))
    headers = {
        "driver/i2c_master.h": "#pragma once\n",
        "sdkconfig.h": "#pragma once\n",
        "freertos/FreeRTOS.h": """#pragma once
#include <assert.h>
#include <stdint.h>
typedef uint32_t TickType_t;
#define portMAX_DELAY 0xffffffff
#define pdMS_TO_TICKS(ms) (ms)
#define configASSERT(x) assert(x)
""",
        "freertos/semphr.h": """#pragma once
typedef void *SemaphoreHandle_t;
static inline SemaphoreHandle_t xSemaphoreCreateMutex(void) { return (void *)1; }
static inline void xSemaphoreTake(SemaphoreHandle_t mutex, unsigned wait) {}
static inline void xSemaphoreGive(SemaphoreHandle_t mutex) {}
""",
        "freertos/task.h": """#pragma once
#include "FreeRTOS.h"
static inline TickType_t xTaskGetTickCount(void) { return 0; }
static inline void vTaskDelayUntil(TickType_t *last, TickType_t ticks) {}
""",
        "esp_log.h": """#pragma once
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGE(...) ((void)0)
#define ESP_LOGD(...) ((void)0)
""",
        "esp_timer.h": """#pragma once
#include <stdint.h>
static int64_t test_clock_us;
static inline int64_t esp_timer_get_time(void) { return test_clock_us; }
""",
        "esp_heap_caps.h": """#pragma once
#include <stdlib.h>
#define MALLOC_CAP_SPIRAM 0
static inline void *heap_caps_calloc(size_t n, size_t size, int caps) { return calloc(n, size); }
""",
    }
    for name, content in headers.items():
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)

    harness = tmp_path / "test_nav.c"
    harness.write_text('#include "' + str(source_dir / "nav_task.c") + '"\n' + r'''
#include <assert.h>

static drone_state_t vehicle;
static float sent_x, sent_y, sent_z, sent_yaw;
static float offset_x = 10, offset_y = -5;
static int sent_count, hold_count;
static bool position_valid = true;

drone_state_t mavlink_get_state(void) { return vehicle; }
bool mavlink_position_valid(void) { return position_valid; }
bool tof_is_healthy(void) { return false; }
tof_scan_collapsed_t tof_get_collapsed_scan(void) { return (tof_scan_collapsed_t){0}; }
wifi_peer_list_t wifi_get_peers(void) { return (wifi_peer_list_t){0}; }
bool wifi_is_connected(void) { return true; }
void mavlink_arm(bool arm) { vehicle.armed = arm; }
void map_to_odom(float x, float y, float *ox, float *oy) { *ox = x-offset_x; *oy = y-offset_y; }
void mavlink_set_hold(void) { hold_count++; }
void mavlink_set_position_ned(float x, float y, float z, float yaw) {
    sent_x = x; sent_y = y; sent_z = z; sent_yaw = yaw; sent_count++;
}
void mavlink_set_position_velocity_ned(float x, float y, float z,
                                      float vx, float vy, float vz, float yaw) {
    mavlink_set_position_ned(x, y, z, yaw);
}
void mavlink_set_velocity_xy_position_z(float vx, float vy, float z, float yaw) {
    assert(0 && "unexpected collision avoidance with no healthy ToF");
}

int main(void) {
    nav_task_init();
    vehicle.heading = 1.23f;
    vehicle.z = -0.5f;
    vfh_config_t cfg = VFH_DEFAULT_CONFIG;
    /* Goals on all four sides, then the starting point (return leg). */
    const float points[][2] = {{-2,0}, {0,2}, {2,0}, {0,-2}, {0,0}};
    for (unsigned i = 0; i < sizeof(points)/sizeof(points[0]); i++) {
        nav_set_goal_map(points[i][0]+offset_x, points[i][1]+offset_y, -0.5f);
        assert(s_nav.state == NAV_FLYING);
        int before = sent_count;
        nav_tick(&cfg);
        assert(sent_count == before+1);
        assert(sent_x == points[i][0] && sent_y == points[i][1] && sent_z == -0.5f);
        assert(isnan(sent_yaw)); /* Keep heading; never request the goal bearing. */
        assert(nav_get_status().state == NAV_FLYING);
        /* At arrival, the final position command must also preserve yaw. */
        vehicle.x = points[i][0]; vehicle.y = points[i][1];
        nav_tick(&cfg);
        assert(nav_get_status().state == NAV_ARRIVED);
        assert(isnan(sent_yaw));
    }
    nav_set_goal_map(15, -5, -0.5f);
    int before = sent_count;
    position_valid = false;
    nav_tick(&cfg);
    assert(sent_count == before);
    position_valid = true;
    nav_cancel();
    assert(nav_get_status().state == NAV_IDLE);
    nav_tick(&cfg);
    assert(sent_count == before);
    assert(hold_count > 0);

    /* Existing timed trajectories still hold the heading captured at start. */
    vehicle.armed = true;
    vehicle.custom_main_mode = PX4_MAIN_MODE_OFFBOARD;
    const float trajectory[][3] = {{0,0,0}, {1,0,0}};
    nav_traj_put(1, 2, 0, trajectory, 2);
    nav_traj_start(1, 50);
    nav_tick(&cfg);
    assert(nav_get_status().state == NAV_TRAJ);
    assert(sent_yaw == 1.23f);
    test_clock_us += 100000;
    nav_tick(&cfg);
    assert(nav_get_status().state == NAV_ARRIVED);
    assert(sent_yaw == 1.23f);
    free(s_traj);
    return 0;
}
''')
    executable = tmp_path / "test_nav"
    compiled = subprocess.run(
        [compiler, "-std=c11", "-Werror=implicit-function-declaration",
         "-I", str(tmp_path), "-I", str(source_dir), "-I", str(main_dir),
         str(harness), "-lm", "-o", str(executable)], capture_output=True, text=True)
    assert compiled.returncode == 0, compiled.stderr
    result = subprocess.run([str(executable)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
