#include "nav_task.h"
#include "mavlink_task.h"
#include "tof_task.h"
#include "vfh.h"
#include "wifi_task.h"
#include "odom.h"       /* map_to_odom() — convert goal each tick */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

#include <math.h>
#include <string.h>

static const char *TAG = "nav";

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---------------------------------------------------------------------------
 * Internal state
 * --------------------------------------------------------------------------- */
typedef struct {
    nav_state_t state;
    float goal_map_x, goal_map_y;   /* goal in MAP frame (north, east)          */
    float goal_z;                   /* NED down (negative = above ground)       */
    float goal_yaw;                 /* absolute NED yaw when goal_has_yaw       */
    float align_x, align_y;          /* fixed odom position during initial yaw   */
    bool  goal_has_yaw;             /* false = automatically face travel bearing */
    bool  goal_aligning_initial;     /* explicit-yaw pre-translation alignment   */
    bool  has_goal;
    float prev_steering_rad;    /* VFH prev input for continuity                */
    uint32_t stuck_count;       /* lifetime stuck events                        */
} nav_internal_t;

static nav_internal_t    s_nav;
static nav_status_t      s_status;
static SemaphoreHandle_t s_mutex;

/* Uploaded trajectory. Only written while not playing (checked under s_mutex). */
typedef struct { float x, y, z, yaw; } traj_pt_t;

static traj_pt_t *s_traj;          /* PSRAM; offsets from first point, NaN x = not received */
static uint16_t   s_traj_n;        /* points in the current upload                */
static uint8_t    s_traj_id;       /* current upload id, 0 = none / already played */
static bool       s_traj_upload_has_yaw;
static uint16_t   s_traj_dt_ms;
static float      s_traj_org[3];   /* odom position where playback started        */
static float      s_traj_yaw;
static bool       s_traj_use_yaw;  /* per-point absolute yaw instead of fixed yaw */
static bool       s_traj_aligning; /* rotate to first yaw before starting clock   */
static int64_t    s_traj_t0_us;

/* ---------------------------------------------------------------------------
 * Helpers
 * --------------------------------------------------------------------------- */

/* Wrap angle to (-π, π] */
static float wrap_pi(float a)
{
    a = fmodf(a + (float)M_PI, 2.0f * (float)M_PI);
    if (a < 0.0f) a += 2.0f * (float)M_PI;
    return a - (float)M_PI;
}

/* Total collapsed scan points (shared by peer injection + collision avoidance) */
#define COLLISION_SCAN_PTS    (TOF_SENSOR_COUNT * TOF_SENSOR_RESO)  /* 64         */

/* ---------------------------------------------------------------------------
 * Inject peer drone positions as virtual obstacles into the collapsed scan.
 *
 * For each peer within VFH range, computes the body-frame bearing and
 * distance, then writes the distance into the corresponding scan bin(s).
 * This makes VFH treat other drones as physical obstacles — the density
 * weighting and growth step handle safety margins automatically.
 * --------------------------------------------------------------------------- */
/* Peer avoidance tuning:
 *
 *  A peer blocks its VFH bin when density >= density_threshold (1.5).
 *  density_at_d = PEER_DENSITY_MAX * (1 − d / PEER_INJECT_RANGE_M)
 *  Blocking distance = PEER_INJECT_RANGE_M * (1 − 1.5/PEER_DENSITY_MAX)
 *
 *  With the values below:  block at 4.0 * (1 − 1.5/9.0) = 3.33 m
 *  Head-on at 0.6 m/s each → 3.33 / 1.2 = 2.8 s warning.
 */
#define PEER_INJECT_RANGE_M   4.0f   /* ignore peers beyond this distance (m)  */
#define PEER_DENSITY_MAX      10.0f   /* density at distance 0 (threshold=1.5)  */
#define PEER_BIN_SPREAD       3      /* inject into ±N VFH bins around bearing */

static void inject_peers_into_histogram(float hist[VFH_BINS],
                                        float drone_x, float drone_y,
                                        float heading)
{
    wifi_peer_list_t peers = wifi_get_peers();

    for (int p = 0; p < peers.count; p++) {
        float odom_x, odom_y;
        map_to_odom(peers.peers[p].map_x, peers.peers[p].map_y, &odom_x, &odom_y);

        float dx   = odom_x - drone_x;
        float dy   = odom_y - drone_y;
        float dist = sqrtf(dx * dx + dy * dy);

        if (dist > PEER_INJECT_RANGE_M || dist < 0.01f) continue;

        /* Density boost: strong when close, fades to zero at PEER_INJECT_RANGE_M */
        float density = PEER_DENSITY_MAX * (1.0f - dist / PEER_INJECT_RANGE_M);

        /* Body-frame bearing to peer */
        float ned_bearing  = atan2f(dy, dx);
        float body_bearing = ned_bearing - heading;
        body_bearing = fmodf(body_bearing, 2.0f * (float)M_PI);
        if (body_bearing < 0.0f) body_bearing += 2.0f * (float)M_PI;

        /* Map to VFH bin and spread across ± PEER_BIN_SPREAD */
        float bin_width_rad = 2.0f * (float)M_PI / (float)VFH_BINS;
        int center_bin = (int)(body_bearing / bin_width_rad) % VFH_BINS;

        for (int d = -PEER_BIN_SPREAD; d <= PEER_BIN_SPREAD; d++) {
            int b = (center_bin + d + VFH_BINS) % VFH_BINS;
            /* Taper: full density at center, half at the edges */
            float taper = 1.0f - 0.5f * (float)abs(d) / (float)(PEER_BIN_SPREAD + 1);
            hist[b] += density * taper;
        }
    }
}

/* ---------------------------------------------------------------------------
 * Emergency collision avoidance
 *
 * Scans all 64 collapsed-scan range points.  If any point is below
 * COLLISION_DANGER_M, computes a repulsion velocity in NED frame and
 * commands it directly, bypassing all normal navigation logic.
 *
 * Returns true if avoidance fired (caller should skip the rest of nav_tick).
 * --------------------------------------------------------------------------- */
#define COLLISION_DANGER_M    0.40f   /* trigger threshold (m)                    */
#define COLLISION_CLEAR_M     0.50f   /* resume normal nav once all points above  */
#define COLLISION_SPEED_MS    0.35f    /* escape velocity magnitude (m/s)          */

static bool s_collision_active = false;

static bool collision_avoid(float goal_z)
{
    (void)goal_z;
    return false;

    if (!mavlink_position_valid()) return false;
    if (!tof_is_healthy()) return false;   /* stale sensors — let nav_tick hold */

    tof_scan_collapsed_t scan = tof_get_collapsed_scan();
    drone_state_t drone = mavlink_get_state();

    /* Determine the threshold: once active, require CLEAR_M before releasing */
    float threshold = s_collision_active ? COLLISION_CLEAR_M : COLLISION_DANGER_M;

    /* Accumulate repulsion vector in body frame.
     * Scan index 0 = front (0°), CW.  Each point covers 360/64 = 5.625°. */
    float repulse_fwd  = 0.0f;   /* body X: positive = forward             */
    float repulse_rght = 0.0f;   /* body Y: positive = right               */
    int   threats      = 0;
    float closest      = 999.0f;

    for (int i = 0; i < COLLISION_SCAN_PTS; i++) {
        if (scan.ranges[i] >= threshold) continue;

        /* Body-frame angle of this scan point (CW from forward, radians) */
        float body_rad = (float)i * (2.0f * (float)M_PI / (float)COLLISION_SCAN_PTS);

        /* Weight: closer = stronger repulsion (1 at 0m, 0 at threshold) */
        float weight = 1.0f - (scan.ranges[i] / threshold);

        /* Push AWAY from this direction (body frame) */
        repulse_fwd  -= weight * cosf(body_rad);
        repulse_rght -= weight * sinf(body_rad);
        threats++;

        if (scan.ranges[i] < closest) closest = scan.ranges[i];
    }

    if (threats == 0) {
        if (s_collision_active) {
            ESP_LOGI(TAG, "Collision clear — resuming nav");
            s_collision_active = false;
        }
        return false;
    }

    /* Check if repulsion vectors nearly cancel (e.g. narrow corridor with
     * walls on both sides).  In that case, let VFH handle navigation
     * instead of oscillating between walls.  Only intervene if the net
     * repulsion has meaningful magnitude relative to the total threat weight. */
    float body_mag = sqrtf(repulse_fwd * repulse_fwd + repulse_rght * repulse_rght);
    if (body_mag < 0.3f && closest > 0.18f) {
        /* Opposing threats nearly cancel and nothing is dangerously close —
         * release collision avoidance so VFH can navigate the corridor. */
        if (s_collision_active) {
            ESP_LOGI(TAG, "Collision: opposing threats cancel (mag=%.2f), releasing to VFH",
                     body_mag);
            s_collision_active = false;
        }
        return false;
    }

    /* Convert body-frame repulsion to NED */
    float cos_h = cosf(drone.heading);
    float sin_h = sinf(drone.heading);
    float repulse_n = repulse_fwd * cos_h - repulse_rght * sin_h;
    float repulse_e = repulse_fwd * sin_h + repulse_rght * cos_h;

    /* Normalise and scale to fixed escape speed */
    float ned_mag = sqrtf(repulse_n * repulse_n + repulse_e * repulse_e);
    if (ned_mag < 1e-6f) return false;

    float vn = COLLISION_SPEED_MS * (repulse_n / ned_mag);
    float ve = COLLISION_SPEED_MS * (repulse_e / ned_mag);

    mavlink_set_velocity_xy_position_z(vn, ve, goal_z, drone.heading);

    if (!s_collision_active) {
        ESP_LOGW(TAG, "COLLISION AVOID — %d threats, closest=%.2fm, escape=(%.2f,%.2f) m/s",
                 threats, closest, vn, ve);
        s_collision_active = true;
    } else {
        ESP_LOGD(TAG, "COLLISION: %d threats, closest=%.2fm, vel=(%.2f,%.2f)",
                 threats, closest, vn, ve);
    }

    return true;
}

/* ---------------------------------------------------------------------------
 * Trajectory playback — linear interpolation between samples, with the
 * segment velocity as feedforward so PX4 tracks without lag.
 * --------------------------------------------------------------------------- */
static void traj_tick(void)
{
    float k    = (float)(esp_timer_get_time() - s_traj_t0_us) / (1000.0f * s_traj_dt_ms);
    int   last = s_traj_n - 1;

    if (k >= (float)last) {
        const traj_pt_t *p = &s_traj[last];
        float yaw = s_traj_use_yaw ? p->yaw : s_traj_yaw;
        mavlink_set_position_ned(s_traj_org[0] + p->x, s_traj_org[1] + p->y,
                                 s_traj_org[2] + p->z, yaw);
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        if (s_nav.state == NAV_TRAJ) {
            s_nav.state    = NAV_ARRIVED;
            s_nav.has_goal = false;
            s_status.state = NAV_ARRIVED;
        }
        xSemaphoreGive(s_mutex);
        ESP_LOGI(TAG, "Trajectory done — holding last point");
        return;
    }

    int   i  = (int)k;
    float a  = k - (float)i;
    float hz = 1000.0f / (float)s_traj_dt_ms;
    const traj_pt_t *p0 = &s_traj[i], *p1 = &s_traj[i + 1];
    float yaw = s_traj_yaw;
    if (s_traj_use_yaw) {
        /* Interpolate through the shortest rotation, including across ±π. */
        yaw = wrap_pi(p0->yaw + a * wrap_pi(p1->yaw - p0->yaw));
    }

    mavlink_set_position_velocity_ned(
        s_traj_org[0] + p0->x + a * (p1->x - p0->x),
        s_traj_org[1] + p0->y + a * (p1->y - p0->y),
        s_traj_org[2] + p0->z + a * (p1->z - p0->z),
        (p1->x - p0->x) * hz, (p1->y - p0->y) * hz, (p1->z - p0->z) * hz,
        yaw);
}

/* ---------------------------------------------------------------------------
 * nav_tick — executed every 100 ms (10 Hz) by nav_task
 * --------------------------------------------------------------------------- */
/* Goal from this tick's snapshot still active (no HOLD / new goal since)?
 * Call with s_mutex held. */
static bool goal_unchanged(const nav_internal_t *nav)
{
    return s_nav.has_goal && s_nav.goal_map_x == nav->goal_map_x
                          && s_nav.goal_map_y == nav->goal_map_y
                          && s_nav.goal_has_yaw == nav->goal_has_yaw
                          && (!nav->goal_has_yaw || s_nav.goal_yaw == nav->goal_yaw);
}

static void nav_tick(const vfh_config_t *vfh_cfg)
{
    /* Snapshot shared state */
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    nav_internal_t nav = s_nav;
    bool traj_aligning = s_traj_aligning;
    xSemaphoreGive(s_mutex);

    /* --- Emergency collision avoidance (pre-empts everything, even idle) --- */
    float hold_z = nav.has_goal ? nav.goal_z : -0.5f;  /* fallback cruise alt */
    if (collision_avoid(hold_z)) return;

    if (nav.state == NAV_TRAJ) {
        traj_tick();
        return;
    }

    /* Nothing to do while idle */
    if (nav.state == NAV_IDLE || !nav.has_goal) return;

    /* Require fresh telemetry */
    if (!mavlink_position_valid()) {
        ESP_LOGW(TAG, "Position invalid — skipping tick");
        return;
    }

    drone_state_t drone = mavlink_get_state();

    /* Explicit-yaw trajectories do not start their playback clock until the
     * vehicle is actually facing the first requested yaw. Keep the first
     * trajectory position fixed during alignment. */
    if (traj_aligning) {
        float yaw_error = wrap_pi(s_traj[0].yaw - drone.heading);
        mavlink_set_position_ned(s_traj_org[0] + s_traj[0].x,
                                 s_traj_org[1] + s_traj[0].y,
                                 s_traj_org[2] + s_traj[0].z,
                                 s_traj[0].yaw);
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        if (s_traj_aligning && s_nav.has_goal) {
            s_status.state = NAV_ROTATING;
            s_status.heading_error_rad = yaw_error;
            if (fabsf(yaw_error) <= NAV_YAW_TOL_RAD) {
                s_traj_aligning = false;
                s_traj_t0_us = esp_timer_get_time();
                s_nav.state = NAV_TRAJ;
                s_status.state = NAV_TRAJ;
                ESP_LOGI(TAG, "Trajectory yaw aligned — playback clock started");
            }
        }
        xSemaphoreGive(s_mutex);
        return;
    }

    /* Map → odom (fixed start offset). */
    float goal_x, goal_y;
    map_to_odom(nav.goal_map_x, nav.goal_map_y, &goal_x, &goal_y);

    /* ---- Horizontal distance to goal ---- */
    float dx   = goal_x - drone.x;
    float dy   = goal_y - drone.y;
    float dist = sqrtf(dx * dx + dy * dy);

    /* atan2(East, North) is NED yaw: 0=North, clockwise positive. Explicit
     * yaw decouples camera direction from the direction of translation. */
    float goal_ned_angle = atan2f(dy, dx);
    float command_yaw = nav.goal_has_yaw ? nav.goal_yaw : goal_ned_angle;
    float heading_error = wrap_pi(command_yaw - drone.heading);

    /* A new explicit-yaw goal aligns in place before translation. Once
     * NAV_FLYING, PX4 maintains the requested yaw without repeatedly stopping
     * the vehicle for small disturbances. */
    if (nav.goal_has_yaw && nav.goal_aligning_initial
            && fabsf(heading_error) > NAV_YAW_TOL_RAD) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        bool ok = goal_unchanged(&nav);
        if (ok) {
            mavlink_set_position_ned(nav.align_x, nav.align_y, nav.goal_z, command_yaw);
            s_status.state = NAV_ROTATING;
            s_status.goal_x = nav.goal_map_x;
            s_status.goal_y = nav.goal_map_y;
            s_status.goal_z = nav.goal_z;
            s_status.dist_to_goal = dist;
            s_status.heading_error_rad = heading_error;
        }
        xSemaphoreGive(s_mutex);
        return;
    }
    if (nav.goal_has_yaw && nav.goal_aligning_initial) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        if (goal_unchanged(&nav)) s_nav.goal_aligning_initial = false;
        xSemaphoreGive(s_mutex);
    }

    /* Altitude is held by PX4's own position controller via the Z-position
     * field in the mixed velocity/position setpoint — nothing to do here. */

    /* ---- Check arrival ---- */
    if (dist < NAV_ARRIVE_RADIUS_M) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        bool ok = goal_unchanged(&nav);
        if (ok) {
            mavlink_set_position_ned(goal_x, goal_y, nav.goal_z,
                                     nav.goal_has_yaw ? command_yaw : drone.heading);
            if (nav.goal_has_yaw && fabsf(heading_error) > NAV_YAW_TOL_RAD) {
                s_nav.state = NAV_ROTATING;
                s_status.state = NAV_ROTATING;
            } else {
                s_nav.state    = NAV_ARRIVED;
                s_nav.has_goal = false;
                s_status.state = NAV_ARRIVED;
            }
            s_status.dist_to_goal = dist;
            s_status.heading_error_rad = heading_error;
        }
        xSemaphoreGive(s_mutex);
        if (ok && (!nav.goal_has_yaw || fabsf(heading_error) <= NAV_YAW_TOL_RAD))
            ESP_LOGI(TAG, "Goal reached (dist=%.2f m)", dist);
        return;
    }

    bool blocked[VFH_BINS] = {false};
    int free_count = VFH_BINS;

    /* ---- Compute updated state ---- */
    nav_state_t new_state       = NAV_FLYING;
    float       new_steering    = 0.0f;
    uint32_t    new_stuck_count = nav.stuck_count;

    /* Direct waypoint mode: command the exact target each tick, under the
     * mutex so a racing HOLD / new goal can't be overwritten. */
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (!goal_unchanged(&nav)) {
        xSemaphoreGive(s_mutex);
        return;
    }
    mavlink_set_position_ned(goal_x, goal_y, nav.goal_z, command_yaw);
    s_nav.state             = new_state;
    s_nav.prev_steering_rad = new_steering;
    s_nav.stuck_count       = new_stuck_count;

    s_status.state             = new_state;
    s_status.goal_x            = nav.goal_map_x;
    s_status.goal_y            = nav.goal_map_y;
    s_status.goal_z            = nav.goal_z;
    s_status.dist_to_goal      = dist;
    s_status.heading_error_rad = heading_error;
    s_status.vfh_steering_rad  = new_steering;
    s_status.free_bins         = free_count;
    s_status.stuck_count       = new_stuck_count;
    for (int b = 0; b < VFH_BINS; b++)
        s_status.vfh_blocked[b] = blocked[b];
    xSemaphoreGive(s_mutex);
}

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------- */

void nav_task_init(void)
{
    memset(&s_nav,    0, sizeof(s_nav));
    memset(&s_status, 0, sizeof(s_status));
    s_nav.state    = NAV_IDLE;
    s_status.state = NAV_IDLE;
    s_mutex = xSemaphoreCreateMutex();
    configASSERT(s_mutex != NULL);

    s_traj = heap_caps_calloc(NAV_TRAJ_MAX_PTS, sizeof(traj_pt_t), MALLOC_CAP_SPIRAM);
    configASSERT(s_traj != NULL);
}

void nav_set_goal_map(float map_x, float map_y, float z)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_nav.goal_map_x      = map_x;
    s_nav.goal_map_y      = map_y;
    s_nav.goal_z          = z;
    s_nav.goal_has_yaw    = false;
    s_nav.goal_aligning_initial = false;
    s_nav.has_goal        = true;
    s_nav.state           = NAV_ROTATING;
    s_traj_aligning       = false;
    s_nav.prev_steering_rad = 0.0f;
    xSemaphoreGive(s_mutex);

    /* Issue hold before nav_task takes over, so PX4 freezes in place
     * during the initial alignment rotation. */
    mavlink_set_hold();

    ESP_LOGI(TAG, "Goal set: map (%.2f, %.2f) z=%.2f", map_x, map_y, z);
}

void nav_set_goal_map_yaw(float map_x, float map_y, float z, float yaw)
{
    if (!isfinite(map_x) || !isfinite(map_y) || !isfinite(z) || !isfinite(yaw)) {
        ESP_LOGW(TAG, "Explicit-yaw goal rejected: non-finite value");
        return;
    }

    drone_state_t st = mavlink_get_state();
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_nav.goal_map_x      = map_x;
    s_nav.goal_map_y      = map_y;
    s_nav.goal_z          = z;
    s_nav.goal_yaw        = wrap_pi(yaw);
    s_nav.align_x         = st.x;
    s_nav.align_y         = st.y;
    s_nav.goal_has_yaw    = true;
    s_nav.goal_aligning_initial = true;
    s_nav.has_goal        = true;
    s_nav.state           = NAV_ROTATING;
    s_traj_aligning       = false;
    s_nav.prev_steering_rad = 0.0f;
    xSemaphoreGive(s_mutex);

    /* nav_tick replaces this with a position hold carrying the requested yaw. */
    mavlink_set_hold();
    ESP_LOGI(TAG, "Explicit-yaw goal: map (%.2f, %.2f) z=%.2f yaw=%.1f deg",
             map_x, map_y, z, yaw * 180.0f / (float)M_PI);
}

void nav_cancel(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool active    = s_nav.has_goal;
    s_nav.state    = NAV_IDLE;
    s_nav.has_goal = false;
    s_nav.goal_aligning_initial = false;
    s_traj_aligning = false;
    s_status.state = NAV_IDLE;
    xSemaphoreGive(s_mutex);

    if (!active) return;   /* keep mission_task's setpoint (e.g. mid-climb) */
    mavlink_set_hold();
    ESP_LOGI(TAG, "Navigation cancelled — holding");
}

void nav_traj_put(uint8_t id, uint16_t total, uint16_t offset,
                  const void *xyz, int n)
{
    if (id == 0 || total < 2 || total > NAV_TRAJ_MAX_PTS || n <= 0
            || offset + n > total) {
        ESP_LOGW(TAG, "Bad trajectory chunk (id=%u total=%u off=%u n=%d)",
                 id, total, offset, n);
        return;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool playing = (s_nav.state == NAV_TRAJ || s_traj_aligning);
    if (!playing) {
        if (id != s_traj_id || total != s_traj_n || s_traj_upload_has_yaw) {
            for (int i = 0; i < total; i++) {
                s_traj[i].x = NAN;
                s_traj[i].yaw = NAN;
            }
            s_traj_id = id;
            s_traj_n  = total;
            s_traj_upload_has_yaw = false;
        }
        const uint8_t *src = (const uint8_t *)xyz;
        for (int i = 0; i < n; i++) {
            /* Input is packed on the wire; memcpy is safe for unaligned floats. */
            memcpy(&s_traj[offset + i].x, src + (size_t)i * 12u, 12u);
            s_traj[offset + i].yaw = NAN;
        }
    }
    xSemaphoreGive(s_mutex);

    if (playing) ESP_LOGW(TAG, "Trajectory chunk ignored — playing");
}

void nav_traj_put_yaw(uint8_t id, uint16_t total, uint16_t offset,
                      const void *xyzyaw, int n)
{
    if (id == 0 || total < 2 || total > NAV_TRAJ_MAX_PTS || n <= 0
            || offset + n > total) {
        ESP_LOGW(TAG, "Bad yaw trajectory chunk (id=%u total=%u off=%u n=%d)",
                 id, total, offset, n);
        return;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool playing = (s_nav.state == NAV_TRAJ || s_traj_aligning);
    if (!playing) {
        if (id != s_traj_id || total != s_traj_n || !s_traj_upload_has_yaw) {
            for (int i = 0; i < total; i++) {
                s_traj[i].x = NAN;
                s_traj[i].yaw = NAN;
            }
            s_traj_id = id;
            s_traj_n  = total;
            s_traj_upload_has_yaw = true;
        }
        const uint8_t *src = (const uint8_t *)xyzyaw;
        for (int i = 0; i < n; i++) {
            memcpy(&s_traj[offset + i], src + (size_t)i * 16u, 16u);
            s_traj[offset + i].yaw = wrap_pi(s_traj[offset + i].yaw);
        }
    }
    xSemaphoreGive(s_mutex);

    if (playing) ESP_LOGW(TAG, "Yaw trajectory chunk ignored — playing/alignment active");
}

static void nav_traj_start_common(uint8_t id, uint16_t dt_ms, bool use_yaw)
{
    drone_state_t st = mavlink_get_state();
    bool flying = st.armed && st.custom_main_mode == PX4_MAIN_MODE_OFFBOARD
                  && mavlink_position_valid();
    const char *err = NULL;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_nav.state == NAV_TRAJ)          err = "already playing";
    else if (s_traj_aligning)             err = "yaw alignment already active";
    else if (id == 0 || id != s_traj_id) err = "unknown upload id";
    else if (dt_ms == 0)                 err = "dt_ms is 0";
    else if (!flying)                    err = "not armed in OFFBOARD";
    else if (s_traj_upload_has_yaw != use_yaw) err = "upload/start format mismatch";
    for (int i = 0; !err && i < s_traj_n; i++) {
        if (!isfinite(s_traj[i].x) || !isfinite(s_traj[i].y)
                || !isfinite(s_traj[i].z)
                || (use_yaw && !isfinite(s_traj[i].yaw))) {
            err = "upload incomplete or non-finite";
        }
    }
    if (!err && use_yaw) {
        float dt_s = (float)dt_ms / 1000.0f;
        float previous_rate = 0.0f;
        for (int i = 1; !err && i < s_traj_n; i++) {
            float rate = wrap_pi(s_traj[i].yaw - s_traj[i - 1].yaw) / dt_s;
            if (fabsf(rate) > NAV_TRAJ_MAX_YAW_RATE_RAD_S) {
                err = "yaw rate exceeds limit";
            } else if (fabsf(rate - previous_rate) / dt_s
                       > NAV_TRAJ_MAX_YAW_ACCEL_RAD_S2) {
                err = "yaw acceleration exceeds limit";
            }
            previous_rate = rate;
        }
        if (!err && fabsf(previous_rate) / dt_s > NAV_TRAJ_MAX_YAW_ACCEL_RAD_S2)
            err = "final yaw deceleration exceeds limit";
    }
    if (!err) {
        s_traj_org[0] = st.x;
        s_traj_org[1] = st.y;
        s_traj_org[2] = st.z;
        s_traj_yaw    = st.heading;
        s_traj_use_yaw = use_yaw;
        s_traj_dt_ms  = dt_ms;
        s_traj_id     = 0;              /* consume: a repeated START cannot replay */
        s_nav.has_goal = true;
        s_nav.goal_has_yaw = false;
        if (use_yaw) {
            s_traj_aligning = true;
            s_nav.state = NAV_ROTATING;
            s_status.state = NAV_ROTATING;
            s_status.heading_error_rad = wrap_pi(s_traj[0].yaw - st.heading);
        } else {
            s_traj_aligning = false;
            s_traj_t0_us = esp_timer_get_time();
            s_nav.state = NAV_TRAJ;
            s_status.state = NAV_TRAJ;
        }
    }
    uint16_t n = s_traj_n;
    xSemaphoreGive(s_mutex);

    if (err) {
        ESP_LOGW(TAG, "Trajectory start refused: %s", err);
    } else {
        ESP_LOGI(TAG, "%s trajectory start: %u pts @ %u ms from odom (%.2f, %.2f, %.2f)",
                 use_yaw ? "Explicit-yaw" : "Fixed-yaw",
                 n, dt_ms, st.x, st.y, st.z);
    }
}

void nav_traj_start(uint8_t id, uint16_t dt_ms)
{
    nav_traj_start_common(id, dt_ms, false);
}

void nav_traj_start_yaw(uint8_t id, uint16_t dt_ms)
{
    nav_traj_start_common(id, dt_ms, true);
}

nav_status_t nav_get_status(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    nav_status_t copy = s_status;
    xSemaphoreGive(s_mutex);
    return copy;
}

/* ---------------------------------------------------------------------------
 * FreeRTOS task — 10 Hz navigation loop
 * --------------------------------------------------------------------------- */

void nav_task(void *arg)
{
    vfh_config_t vfh_cfg         = VFH_DEFAULT_CONFIG;
    TickType_t   last_wake        = xTaskGetTickCount();
    TickType_t   wifi_discon_tick = 0;   /* 0 = link up; else tick of first drop */
    bool         wifi_kill_sent   = false;

    ESP_LOGI(TAG, "Navigator started (%.1f m/s cruise, ±%.0f° yaw tol)",
             NAV_CRUISE_SPEED_MS, NAV_YAW_TOL_RAD * 180.0f / (float)M_PI);

    while (1) {
        /* ---- WiFi link-loss killswitch ----
         * Immediately cancel navigation and hold position when WiFi drops.
         * If the link stays down for >3 s while armed, disarm (motor cut). */
        if (!wifi_is_connected()) {
            if (wifi_discon_tick == 0) {
                wifi_discon_tick = xTaskGetTickCount();
                nav_cancel();   /* stop any goal/trajectory */
                ESP_LOGW(TAG, "WiFi link lost — holding position, disarm in 3 s");
            } else if (!wifi_kill_sent &&
                       (xTaskGetTickCount() - wifi_discon_tick) >= pdMS_TO_TICKS(3000) &&
                       mavlink_get_state().armed) {
                ESP_LOGE(TAG, "WiFi lost >3 s — motor kill (disarm)");
                mavlink_arm(false);
                wifi_kill_sent = true;
            }
        } else {
            if (wifi_discon_tick != 0) {
                ESP_LOGI(TAG, "WiFi link restored");
                wifi_discon_tick = 0;
                wifi_kill_sent   = false;
            }
        }

        /* Skip navigation while WiFi is down */
        if (wifi_discon_tick != 0) {
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(50));
            continue;
        }

        nav_tick(&vfh_cfg);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(50));
    }
}
