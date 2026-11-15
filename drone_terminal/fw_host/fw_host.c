/*
 * fw_host.c — runs the drone's real navigation firmware on the laptop.
 *
 * main/nav_task.c is compiled unchanged (it is #included below so the
 * simulator can also read its private playback state), together with
 * main/vfh.c and main/odom.c.  Everything it talks to on the drone is
 * replaced by a small stand-in here:
 *
 *   FreeRTOS time  -> the simulator's clock; nav_task()'s own loop runs on a
 *                     thread that advances in lockstep with the simulation
 *   mavlink_task   -> setpoint setters copied from main/mavlink_task.c
 *                     (same speed clamp); telemetry comes from the simulator
 *   tof_task       -> no sensors (TOF_ENABLED is 0 on the drone too)
 *   wifi_task      -> link up, no peers
 *   ESP_LOGx       -> collected so the simulator can print them
 *
 * So any change to nav_task.c / vfh.c / odom.c (or their headers) changes the
 * simulation; drone_terminal/firmware_host.py rebuilds this automatically.
 */
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "nav_task.c"          /* the real firmware source, from main/ */

#define FW_ABI_VERSION 1

/* ---------------------------------------------------------------------------
 * Simulated clock + lockstep with nav_task()'s thread
 * --------------------------------------------------------------------------- */
#ifdef _WIN32
#include <windows.h>
static CRITICAL_SECTION   g_cs;
static CONDITION_VARIABLE g_cv;
#define LOCK()      EnterCriticalSection(&g_cs)
#define UNLOCK()    LeaveCriticalSection(&g_cs)
#define WAIT()      SleepConditionVariableCS(&g_cv, &g_cs, INFINITE)
#define BROADCAST() WakeAllConditionVariable(&g_cv)
#else
#include <pthread.h>
static pthread_mutex_t g_mx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_cv = PTHREAD_COND_INITIALIZER;
#define LOCK()      pthread_mutex_lock(&g_mx)
#define UNLOCK()    pthread_mutex_unlock(&g_mx)
#define WAIT()      pthread_cond_wait(&g_cv, &g_mx)
#define BROADCAST() pthread_cond_broadcast(&g_cv)
#endif

static int64_t g_now_us;        /* simulated time, never reset             */
static int64_t g_wake_us;       /* nav thread sleeps until this time       */
static int     g_parked;        /* nav thread is waiting in vTaskDelayUntil */
static int     g_started;
static int     g_ticks;         /* nav loop iterations, for the simulator  */

int64_t esp_timer_get_time(void) { return g_now_us; }
TickType_t xTaskGetTickCount(void) { return (TickType_t)(g_now_us / 1000); }

void vTaskDelayUntil(TickType_t *previous_wake, TickType_t increment)
{
    *previous_wake += increment;
    LOCK();
    g_wake_us = (int64_t)(*previous_wake) * 1000;
    g_parked = 1;
    g_ticks++;
    BROADCAST();
    while (g_now_us < g_wake_us) WAIT();
    g_parked = 0;
    UNLOCK();
}

void vTaskDelay(TickType_t ticks)
{
    TickType_t now = xTaskGetTickCount();
    vTaskDelayUntil(&now, ticks);
}

/* ---------------------------------------------------------------------------
 * Log capture
 * --------------------------------------------------------------------------- */
static char g_log[32768];
static size_t g_log_len;

void fw_log(char level, const char *tag, const char *fmt, ...)
{
    char line[512];
    int n = snprintf(line, sizeof line, "%c %.3f [%s] ", level,
                     (double)g_now_us / 1e6, tag);
    va_list ap;
    va_start(ap, fmt);
    n += vsnprintf(line + n, sizeof line - (size_t)n, fmt, ap);
    va_end(ap);
    if (n > (int)sizeof line - 2) n = (int)sizeof line - 2;
    line[n++] = '\n';
    if (g_log_len + (size_t)n < sizeof g_log) {
        memcpy(g_log + g_log_len, line, (size_t)n);
        g_log_len += (size_t)n;
    }
}

/* ---------------------------------------------------------------------------
 * mavlink_task stand-in (setters mirror main/mavlink_task.c)
 * --------------------------------------------------------------------------- */
enum { FW_SP_HOLD = 0, FW_SP_POSITION, FW_SP_VELOCITY, FW_SP_VEL_XY_POS_Z, FW_SP_POS_VEL };

typedef struct {
    int32_t type;
    float x, y, z, vx, vy, vz, yaw, yaw_rate;
} fw_setpoint_t;

static drone_state_t g_state;
static bool          g_pos_valid = true;
static fw_setpoint_t g_sp;
static int           g_arm_requests;   /* mavlink_arm() calls, e.g. killswitch */

static void clamp2(float *vx, float *vy, float max_speed)
{
    float mag = sqrtf((*vx) * (*vx) + (*vy) * (*vy));
    if (mag <= max_speed || mag < 1e-6f) return;
    float scale = max_speed / mag;
    *vx *= scale;
    *vy *= scale;
}

static void clamp3(float *vx, float *vy, float *vz, float max_speed)
{
    float mag = sqrtf((*vx) * (*vx) + (*vy) * (*vy) + (*vz) * (*vz));
    if (mag <= max_speed || mag < 1e-6f) return;
    float scale = max_speed / mag;
    *vx *= scale;
    *vy *= scale;
    *vz *= scale;
}

void mavlink_set_velocity_ned(float vx, float vy, float vz, float yaw_rate)
{
    clamp3(&vx, &vy, &vz, MAV_CMD_SPEED_CAP_MS);
    g_sp.type = FW_SP_VELOCITY;
    g_sp.vx = vx; g_sp.vy = vy; g_sp.vz = vz; g_sp.yaw_rate = yaw_rate;
}

void mavlink_set_velocity_xy_position_z(float vx, float vy, float z, float yaw)
{
    clamp2(&vx, &vy, MAV_CMD_SPEED_CAP_MS);
    g_sp.type = FW_SP_VEL_XY_POS_Z;
    g_sp.vx = vx; g_sp.vy = vy; g_sp.z = z; g_sp.yaw = yaw;
}

void mavlink_set_position_ned(float x, float y, float z, float yaw)
{
    g_sp.type = FW_SP_POSITION;
    g_sp.x = x; g_sp.y = y; g_sp.z = z; g_sp.yaw = yaw;
}

void mavlink_set_position_velocity_ned(float x, float y, float z,
                                       float vx, float vy, float vz, float yaw)
{
    clamp3(&vx, &vy, &vz, MAV_CMD_SPEED_CAP_MS);
    g_sp.type = FW_SP_POS_VEL;
    g_sp.x = x; g_sp.y = y; g_sp.z = z;
    g_sp.vx = vx; g_sp.vy = vy; g_sp.vz = vz; g_sp.yaw = yaw;
}

void mavlink_set_hold(void)
{
    g_sp.type = FW_SP_HOLD;          /* PX4 keeps its current heading */
    g_sp.x = g_state.x; g_sp.y = g_state.y; g_sp.z = g_state.z;
    g_sp.yaw = NAN;
}

void mavlink_arm(bool arm) { g_state.armed = arm; g_arm_requests++; }
drone_state_t mavlink_get_state(void) { return g_state; }
bool mavlink_position_valid(void) { return g_pos_valid; }

/* ---------------------------------------------------------------------------
 * tof_task / wifi_task stand-ins
 * --------------------------------------------------------------------------- */
static bool g_wifi_ok = true;

bool tof_is_healthy(void) { return false; }
tof_scan_collapsed_t tof_get_collapsed_scan(void)
{
    tof_scan_collapsed_t scan;
    memset(&scan, 0, sizeof scan);
    return scan;
}
bool wifi_is_connected(void) { return g_wifi_ok; }
wifi_peer_list_t wifi_get_peers(void)
{
    wifi_peer_list_t list;
    memset(&list, 0, sizeof list);
    return list;
}

/* ---------------------------------------------------------------------------
 * Simulator API (called from Python while the nav thread is parked)
 * --------------------------------------------------------------------------- */
#ifdef _WIN32
static DWORD WINAPI nav_thread(LPVOID arg) { nav_task(arg); return 0; }
#else
static void *nav_thread(void *arg) { nav_task(arg); return NULL; }
#endif

int fw_abi(void) { return FW_ABI_VERSION; }

static void reset_state(void)
{
    memset(&g_state, 0, sizeof g_state);
    g_state.armed = true;
    g_state.custom_main_mode = PX4_MAIN_MODE_OFFBOARD;
    g_pos_valid = true;
    g_wifi_ok = true;
    memset(&g_sp, 0, sizeof g_sp);
    g_sp.yaw = NAN;
    g_arm_requests = 0;
    g_log_len = 0;
}

/* First call starts nav_task() on its own thread; later calls reset the
 * firmware's state between simulated flights (the thread keeps running). */
void fw_reset(void)
{
    if (!g_started) {
#ifdef _WIN32
        InitializeCriticalSection(&g_cs);
        InitializeConditionVariable(&g_cv);
#endif
        reset_state();
        odom_init();
        nav_task_init();
        g_started = 1;
        LOCK();
        g_wake_us = INT64_MAX;
#ifdef _WIN32
        CreateThread(NULL, 0, nav_thread, NULL, 0, NULL);
#else
        pthread_t th;
        pthread_create(&th, NULL, nav_thread, NULL);
        pthread_detach(th);
#endif
        while (!g_parked) WAIT();
        UNLOCK();
        g_log_len = 0;
        return;
    }
    free(s_traj);
    s_traj = NULL;
    s_traj_n = 0;
    s_traj_id = 0;
    s_traj_upload_has_yaw = false;
    s_traj_aligning = false;
    s_collision_active = false;
    reset_state();
    nav_task_init();
}

void fw_set_state(float x, float y, float z, float vx, float vy, float vz,
                  float heading, int armed, int pos_valid, int wifi_ok)
{
    uint32_t ms = (uint32_t)(g_now_us / 1000);
    g_state.x = x; g_state.y = y; g_state.z = z;
    g_state.vx = vx; g_state.vy = vy; g_state.vz = vz;
    g_state.heading = heading;
    g_state.enu_z = -z;
    g_state.armed = armed != 0;
    g_state.custom_main_mode = PX4_MAIN_MODE_OFFBOARD;
    g_state.last_pos_ms = g_state.last_att_ms = g_state.last_hb_ms = ms;
    g_pos_valid = pos_valid != 0;
    g_wifi_ok = wifi_ok != 0;
}

/* Advance simulated time; runs every nav_task() loop iteration that falls due.
 * Returns how many iterations ran. */
int fw_advance_us(int64_t dt_us)
{
    LOCK();
    int start = g_ticks;
    g_now_us += dt_us;
    while (g_wake_us <= g_now_us) {        /* nav_task() is due */
        int t = g_ticks;
        BROADCAST();
        while (g_ticks == t) WAIT();       /* one loop iteration, parked again */
    }
    int ran = g_ticks - start;
    UNLOCK();
    return ran;
}

void fw_get_setpoint(fw_setpoint_t *out) { *out = g_sp; }
int fw_nav_state(void) { return (int)s_nav.state; }
int fw_traj_aligning(void) { return s_traj_aligning ? 1 : 0; }
int fw_arm_requests(void) { return g_arm_requests; }

/* Firmware playback position as a fractional point index, or -1. */
double fw_traj_progress(void)
{
    if (s_nav.state != NAV_TRAJ || s_traj_dt_ms == 0) return -1.0;
    return (double)(g_now_us - s_traj_t0_us) / (1000.0 * s_traj_dt_ms);
}

int fw_take_log(char *buf, int size)
{
    int n = (int)g_log_len < size - 1 ? (int)g_log_len : size - 1;
    memcpy(buf, g_log, (size_t)n);
    buf[n] = '\0';
    memmove(g_log, g_log + n, g_log_len - (size_t)n);
    g_log_len -= (size_t)n;
    return n;
}
