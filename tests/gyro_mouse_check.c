// Player space gyro (runtime/gyro_mouse.c) with the rest of the runtime stubbed: the yaw a turn about the vertical makes
// with the controller held flat, upright and tilted between; the yaw a sideways roll adds; pitch; the sensitivity; the
// bias a still controller loses; slow noise held back; the turn dropped while nothing takes it; the setting gates.
// Built by tests/gyro_mouse_check.py.
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "gyro_mouse.c"

static const char *mode = "player", *scope = "always";
static double sens_value = 1;
static unsigned version = 1;
static int motion_on = 1, patch_on = 1;
static uint64_t clock_ns = 1000000000ull;
unsigned cfg_version(void) { return version; }
const char *cfg_value(const char *name) {
    return !strcmp(name, "NX2NAVI_GYRO_MODE") ? mode : !strcmp(name, "NX2NAVI_GYRO_SCOPE") ? scope : "";
}
double cfg_double(const char *name) { return strcmp(name, "NX2NAVI_GYRO_SENS") ? 0 : sens_value; }
int input_motion_enabled(void) { return motion_on; }
int input_direct_mouse_available(void) { return patch_on; }
uint64_t plat_now_ns(void) { return clock_ns; }
void plat_mutex_lock(PlatMutex *m) { (void)m; }
void plat_mutex_unlock(PlatMutex *m) { (void)m; }

#define DEG (M_PI / 180)
static int near(double a, double b, double tol) { return fabs(a - b) <= tol; }

// One second of 250 Hz samples of rate w (rad/s) with this up direction and gravity on the accelerometer, then a take.
static void run(const double w[3], const double up[3], double seconds, double deg[2]) {
    double acc[3] = {-up[0], -up[1], -up[2]};
    for (int i = 0; i < (int)(seconds * 250 + 0.5); i++) {
        clock_ns += 4000000;
        gyro_mouse_feed(acc, w, up);
        if (i % 4 == 3) {   // the camera takes at 60-ish Hz; sum what it got
            double d[2];
            gyro_mouse_take(d);
            deg[0] += d[0], deg[1] += d[1];
        }
    }
}

int main(void) {
    double out[2];
    const double flat[3] = {0, 0, 1}, upright[3] = {0, 1, 0};
    const double tilt[3] = {0, sqrt(0.5), sqrt(0.5)};   // 45 degrees between

    // a turn about the vertical, 1 rad/s to the left (counter-clockwise from above), is the same yaw however it is held
    gyro_player_space((const double[3]){0, 0, 1}, flat, out);
    assert(near(out[0], -1, 1e-9) && near(out[1], 0, 1e-9));
    gyro_player_space((const double[3]){0, 1, 0}, upright, out);
    assert(near(out[0], -1, 1e-9));
    gyro_player_space((const double[3]){0, sqrt(0.5), sqrt(0.5)}, tilt, out);
    assert(near(out[0], -1, 1e-9));
    // held flat, a sideways roll (about the top edge's axis) adds yaw, up to the relaxed share of the combined rate
    gyro_player_space((const double[3]){0, 1, 0}, tilt, out);
    assert(near(out[0], -sqrt(0.5) * 1.41, 1e-9));   // let out 1.41 times, just under the combined rate 1
    gyro_player_space((const double[3]){0, 1, 0.3}, tilt, out);
    assert(near(out[0], -hypot(1, 0.3), 1e-9));        // here the combined rate is the limit
    gyro_player_space((const double[3]){0, 0.2, 0}, flat, out);
    assert(near(out[0], 0, 1e-9));    // flat, a pure roll is not a turn about the vertical
    // pitch is the x axis
    gyro_player_space((const double[3]){0.5, 0, 0}, upright, out);
    assert(near(out[1], 0.5, 1e-9) && near(out[0], 0, 1e-9));

    // the gates
    assert(gyro_mouse_on());
    motion_on = 0, version++;
    assert(!gyro_mouse_on());
    motion_on = 1, patch_on = 0, version++;
    assert(!gyro_mouse_on());
    patch_on = 1, mode = "game", version++;
    assert(!gyro_mouse_on());
    mode = "player", version++;
    assert(gyro_mouse_on() && !gyro_mouse_aim_only());
    scope = "aim", version++;
    assert(gyro_mouse_on() && gyro_mouse_aim_only());
    scope = "always", version++;

    // feed and take: 30 degrees a second to the right for a second, held upright
    double deg[2] = {0, 0}, warm[2] = {0, 0};
    run((const double[3]){0, 0, 0}, upright, 0.1, warm);   // the first take only starts the count
    run((const double[3]){0, -30 * DEG, 0}, upright, 1, deg);
    assert(near(deg[0], 30, 0.5) && near(deg[1], 0, 1e-6));
    // twice the sensitivity, twice the turn; and 20 degrees up
    sens_value = 2, version++;
    deg[0] = deg[1] = 0;
    run((const double[3]){20 * DEG, -30 * DEG, 0}, upright, 1, deg);
    assert(near(deg[0], 60, 1) && near(deg[1], 40, 1));
    sens_value = 1, version++;

    // a still controller with a bias of 1 degree a second: two thirds of it held back at once (under 1.5 degrees a
    // second), and learnt within a few seconds
    deg[0] = deg[1] = 0;
    run((const double[3]){0, 0, 1 * DEG}, flat, 1, deg);
    assert(fabs(deg[0]) < 0.7);
    run((const double[3]){0, 0, 1 * DEG}, flat, 8, warm);
    assert(near(bias[2], 1 * DEG, 0.1 * DEG));
    deg[0] = deg[1] = 0;
    run((const double[3]){0, 0, 1 * DEG}, flat, 2, deg);
    assert(fabs(deg[0]) < 0.05);
    // ... and a real turn is not taken for bias
    deg[0] = deg[1] = 0;
    run((const double[3]){0, 0, 1 * DEG + 45 * DEG}, flat, 1, deg);
    assert(near(deg[0], -45, 1) && near(bias[2], 1 * DEG, 0.1 * DEG));

    // nothing takes for a while (a menu): what was turned meanwhile is dropped, not added at once afterwards
    double acc[3] = {0, 0, -1}, w[3] = {0, 0, 1 * DEG + 90 * DEG};
    for (int i = 0; i < 100; i++) clock_ns += 4000000, gyro_mouse_feed(acc, w, flat);
    double d[2];
    gyro_mouse_take(d);
    assert(d[0] == 0 && d[1] == 0);

    puts("gyro_mouse_check: ok");
    return 0;
}
