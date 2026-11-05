#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "mavlink_task.h"
#include "tof_task.h"
#include "nav_task.h"
#include "at_detect.h"
#include "wifi_task.h"
#include "odom.h"
#include "takeoff_profile.h"

#include <math.h>

static const char *TAG = "mission";

/* ---------------------------------------------------------------------------
 * Test parameters — adjust before flight
 * --------------------------------------------------------------------------- */
#define ARM_TIMEOUT_S       10      /* give up on OFFBOARD + arm (s)            */
#define LAND_TIMEOUT_S      20      /* warn if still armed after landing (s)    */

/* Consume a pending CMD_LAND. */
static bool land_requested(void)
{
    if (!wifi_land_requested()) return false;
    wifi_clear_land_request();
    ESP_LOGI(TAG, "CMD_LAND received from laptop");
    return true;
}

/* ---------------------------------------------------------------------------
 * Mission task: START → OFFBOARD → arm → takeoff → laptop control → LAND,
 * repeat.  Owns the setpoint until a GOTO/trajectory; nav_cancel() returns it.
 * --------------------------------------------------------------------------- */
static void mission_task(void *arg)
{
    (void)arg;
    for (;;) {
        wifi_set_mission_phase(MISSION_BUSY);
        ESP_LOGI(TAG, "Waiting for fresh telemetry and a disarmed drone...");
        while (!mavlink_position_valid() || mavlink_get_state().armed) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        nav_cancel();
        wifi_clear_start_request();
        wifi_clear_land_request();
        /* Refresh the ground hold instead of retaining an old estimator pose. */
        for (int i = 0; i < 20; i++) {
            mavlink_set_hold();
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (!mavlink_position_valid() || mavlink_get_state().armed) continue;
        wifi_set_mission_phase(MISSION_READY);
        ESP_LOGI(TAG, "Waiting for CMD_START from laptop...");
        while (!wifi_start_requested() && mavlink_position_valid()
               && !mavlink_get_state().armed) {
            mavlink_set_hold();
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (!wifi_start_requested() || !mavlink_position_valid()
                || mavlink_get_state().armed) continue;
        wifi_clear_start_request();
        wifi_set_mission_phase(MISSION_BUSY);
        const char *abort_why = NULL;

#if TOF_ENABLED
        int64_t sensor_deadline = esp_timer_get_time() + ARM_TIMEOUT_S * 1000000LL;
        while (tof_sensors_ok_count() < TOF_SENSOR_COUNT) {
            if (land_requested() || !mavlink_position_valid()
                    || esp_timer_get_time() >= sensor_deadline) {
                abort_why = "ToF pre-arm check cancelled or timed out";
                break;
            }
            mavlink_set_hold();
            vTaskDelay(pdMS_TO_TICKS(100));
        }
#endif

        int64_t arm_deadline = esp_timer_get_time() + ARM_TIMEOUT_S * 1000000LL;
        mavlink_set_hold();
        while (!abort_why && mavlink_get_state().custom_main_mode != PX4_MAIN_MODE_OFFBOARD) {
            if (land_requested() || !mavlink_position_valid()
                    || esp_timer_get_time() >= arm_deadline) {
                abort_why = "OFFBOARD cancelled, stale, or refused";
                break;
            }
            mavlink_set_hold();
            mavlink_set_offboard_mode();
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        drone_state_t ground = mavlink_get_state();
        float ground_z = ground.z;
        float takeoff_x = ground.x, takeoff_y = ground.y;
        while (!abort_why && !mavlink_get_state().armed) {
            if (land_requested() || !mavlink_position_valid()
                    || mavlink_get_state().custom_main_mode != PX4_MAIN_MODE_OFFBOARD
                    || esp_timer_get_time() >= arm_deadline) {
                abort_why = "Arming cancelled, stale, or refused";
                break;
            }
            /* Capture immediately before each arm attempt, without assuming
             * that local NED z is zero on the ground. */
            mavlink_set_hold();
            ground = mavlink_get_state();
            ground_z = ground.z;
            takeoff_x = ground.x;
            takeoff_y = ground.y;
            wifi_set_ground_z(ground_z);
            mavlink_arm(true);
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        bool settled = false;
        if (!abort_why) {
            int64_t ramp_start = esp_timer_get_time();
            int64_t stable_since = -1;
            ESP_LOGI(TAG, "Takeoff: ground z=%.2f, target z=%.2f, ramp %.1fs",
                     ground_z, ground_z - TAKEOFF_HEIGHT_M, TAKEOFF_RAMP_S);
            while (esp_timer_get_time() - ramp_start < TAKEOFF_TIMEOUT_S * 1000000LL) {
                drone_state_t st = mavlink_get_state();
                if (land_requested() || !st.armed || !mavlink_position_valid()
                        || st.custom_main_mode != PX4_MAIN_MODE_OFFBOARD) {
                    abort_why = "Takeoff cancelled, feedback lost, or RC takeover";
                    break;
                }
                int64_t now = esp_timer_get_time();
                float elapsed = (float)(now - ramp_start) / 1000000.0f;
                /* NaN yaw plus YAW_IGNORE holds the existing heading in PX4. */
                mavlink_set_position_ned(takeoff_x, takeoff_y,
                                         takeoff_target_z(ground_z, elapsed), NAN);
                if (elapsed >= TAKEOFF_RAMP_S && takeoff_at_hover(st.z, st.vz, ground_z)) {
                    if (stable_since < 0) stable_since = now;
                    if ((float)(now - stable_since) / 1000000.0f >= TAKEOFF_SETTLE_S) {
                        settled = true;
                        break;
                    }
                } else {
                    stable_since = -1;
                }
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            if (!settled && !abort_why) abort_why = "Takeoff hover timeout";
        }

        if (settled) {
            wifi_set_mission_phase(MISSION_FLYING);
            ESP_LOGI(TAG, "Verified hover: waiting for laptop goals...");
            while (!land_requested()) {
                drone_state_t st = mavlink_get_state();
                if (!st.armed || !mavlink_position_valid()
                        || st.custom_main_mode != PX4_MAIN_MODE_OFFBOARD) break;
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
        if (abort_why) ESP_LOGW(TAG, "Takeoff aborted: %s", abort_why);
        wifi_set_mission_phase(MISSION_BUSY);
        nav_cancel();

        /* Never force-disarm an uncertain arm attempt or override RC takeover.
         * Request landing only while PX4 is still in OFFBOARD. */
        for (int i = 0; mavlink_get_state().armed; i++) {
            if (i % 20 == 0
                    && mavlink_get_state().custom_main_mode == PX4_MAIN_MODE_OFFBOARD) {
                mavlink_send_land_command();
            }
            if (i == LAND_TIMEOUT_S * 10) {
                ESP_LOGW(TAG, "Still armed after %d s: land with RC", LAND_TIMEOUT_S);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        ESP_LOGI(TAG, "Disarmed: waiting for next CMD_START");
    }
}

/* ---------------------------------------------------------------------------
 * app_main — init and spawn all tasks
 * --------------------------------------------------------------------------- */
void app_main(void)
{
    ESP_LOGI(TAG, "ESP starting");

    /* NVS must be init before WiFi */
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    /* Init all modules before spawning tasks */
    mavlink_task_init();
    tof_task_init();    /* always: creates the scan mutex the tof_get_*() accessors take */
    nav_task_init();
    at_detect_init();
    odom_init();
    wifi_task_init();   /* blocks until IP obtained */

    /* Core 0: hardware-facing tasks + WiFi telemetry */
    xTaskCreatePinnedToCore(
        mavlink_task, "mav", MAV_TASK_STACK,
        NULL, MAV_TASK_PRIORITY, NULL, MAV_TASK_CORE
    );
#if TOF_ENABLED
    xTaskCreatePinnedToCore(
        tof_task, "tof", TOF_TASK_STACK,
        NULL, TOF_TASK_PRIORITY, NULL, TOF_TASK_CORE
    );
#endif
    xTaskCreatePinnedToCore(
        wifi_task, "wifi", WIFI_TASK_STACK,
        NULL, WIFI_TASK_PRIORITY, NULL, WIFI_TASK_CORE
    );

    /* Core 1: navigator + AprilTag + mission
     * nav_task preempts mission_task on Core 1 when it has work. */
    xTaskCreatePinnedToCore(
        nav_task, "nav", NAV_TASK_STACK,
        NULL, NAV_TASK_PRIORITY, NULL, NAV_TASK_CORE
    );
    xTaskCreatePinnedToCore(
        at_detect_task, "apriltag", AT_TASK_STACK,
        NULL, AT_TASK_PRIORITY, NULL, AT_TASK_CORE
    );
    xTaskCreatePinnedToCore(
        mission_task, "mission", 6144,
        NULL, 2, NULL, 1
    );

    ESP_LOGI(TAG, "All tasks spawned");
}
