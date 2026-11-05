#pragma once

#include <math.h>
#include <stdbool.h>

#define TAKEOFF_HEIGHT_M       0.5f
#define TAKEOFF_RAMP_S         5.0f
#define TAKEOFF_TIMEOUT_S      20
#define TAKEOFF_SETTLE_S       1.0f
#define TAKEOFF_ALT_TOL_M      0.08f
#define TAKEOFF_VZ_TOL_MS      0.10f

/* Ground-relative smoothstep: zero endpoint velocity, peak climb 0.15 m/s. */
static inline float takeoff_target_z(float ground_z, float elapsed_s)
{
    float u = fminf(1.0f, fmaxf(0.0f, elapsed_s / TAKEOFF_RAMP_S));
    return ground_z - TAKEOFF_HEIGHT_M * u * u * (3.0f - 2.0f * u);
}

static inline bool takeoff_at_hover(float z, float vz, float ground_z)
{
    return isfinite(z) && isfinite(vz) && isfinite(ground_z)
        && fabsf(z - (ground_z - TAKEOFF_HEIGHT_M)) < TAKEOFF_ALT_TOL_M
        && fabsf(vz) < TAKEOFF_VZ_TOL_MS;
}
