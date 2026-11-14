// Player space gyro (NX2NAVI_GYRO_MODE=player, live): the gamepad's gyro turns the camera through the mouse camera's
// path (mouse_direct.c camera_input_oot) instead of reaching the game as its six-axis sensor. Yaw follows the turn
// about the real vertical, wherever the controller points: the turn about gravity, measured on the controller's two
// axes other than its pitch axis (x), then let out up to 1.41 times its size but never past the combined rate of those
// two axes, so a turn the hands make by rolling the controller sideways counts as well as one made by twisting it
// flat. Pitch is the turn about the controller's own x axis. Holding the controller flat, upright or anywhere between
// then turns the camera the same way for the same movement of the hands.
//
// The sensor thread (input.c motion_main, 250 Hz) feeds each sample with the fusion's up direction
// (motion_fusion.c motion_up); the camera input update takes the turn gathered since its last call, in degrees.
// NX2NAVI_GYRO_SCOPE (live): always, or aim, only while Link is in a state the game's own gyro aiming works in
// (mouse_direct.c link_aiming).
// While the game's six-axis sensor would read it, the sensor reads at rest instead (ov_input.c six_fill), so the
// game's own gyro aiming does not add a second turn.
//
// A still controller is not quite still in its readings: its gyro has a small bias. While the controller lies or is
// held still (gravity alone on the accelerometer, the rate under STILL_RATE) for STILL_SECONDS, the rate is averaged into
// the bias, which every sample loses. Rates under TIGHTEN_DEG_S are scaled down in proportion to their size, so what
// remains of the noise does not creep the camera; a deliberate turn is far faster.
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "cfg.h"
#include "gyro_mouse.h"
#include "input.h"
#include "platform/platform.h"

#define YAW_RELAX 1.41
#define STILL_RATE 0.06        // rad/s (3.4 degrees a second)
#define STILL_ACC 0.03         // G off 1
#define STILL_SECONDS 1.0
#define BIAS_SECONDS 2.0       // the bias average's time constant once still
#define TIGHTEN_DEG_S 1.5
#define STALE_NS 100000000ull  // no take for this long (a menu, a load): the turn gathered meanwhile is dropped
#define RAD_DEG (180.0 / M_PI)

void gyro_player_space(const double w[3], const double up[3], double out[2]) {
    double n = sqrt(up[0] * up[0] + up[1] * up[1] + up[2] * up[2]);
    double u1 = n > 0 ? up[1] / n : 0, u2 = n > 0 ? up[2] / n : 1;
    double world = w[1] * u1 + w[2] * u2, limit = hypot(w[1], w[2]);
    double yaw = fmin(fabs(world) * YAW_RELAX, limit);
    out[0] = -copysign(yaw, world);   // a turn to the left (counter-clockwise from above) turns the camera left
    out[1] = w[0];                    // the top edge tilting up looks up
}

static int mode_player = -1, scope_aim;
static double sens = 2;
static unsigned seen;

static void settings(void) {
    unsigned v = cfg_version();
    if (mode_player >= 0 && v == seen) return;
    seen = v;
    const char *m = cfg_value("NX2NAVI_GYRO_MODE");
    mode_player = m && !strcmp(m, "player");
    const char *g = cfg_value("NX2NAVI_GYRO_SCOPE");
    scope_aim = g && !strcmp(g, "aim");
    double s = cfg_double("NX2NAVI_GYRO_SENS");
    sens = s < 0 ? 0 : s > 10 ? 10 : s;
}

int gyro_mouse_on(void) {
    settings();
    return mode_player && input_motion_enabled() && input_direct_mouse_available();
}

int gyro_mouse_aim_only(void) {
    settings();
    return scope_aim;
}

static PlatMutex lock = PLAT_MUTEX_INIT;
static double pending[2];   // degrees: right, up
static double bias[3], still_s;
static uint64_t last_feed, last_take;

void gyro_mouse_feed(const double acc[3], const double gyro[3], const double up[3]) {
    uint64_t now = plat_now_ns();
    plat_mutex_lock(&lock);
    double dt = last_feed && now > last_feed ? (double)(now - last_feed) / 1e9 : 0;
    if (dt > 0.05) dt = 0;   // a gap: nothing turns across it
    last_feed = now;
    double a = sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]), w[3];
    for (int i = 0; i < 3; i++) w[i] = gyro[i] - bias[i];
    double rate = sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
    if (fabs(a - 1) < STILL_ACC && rate < STILL_RATE) {
        still_s += dt;
        if (still_s > STILL_SECONDS && dt > 0) {
            double k = dt / BIAS_SECONDS;
            for (int i = 0; i < 3; i++) bias[i] += k * w[i], w[i] -= k * w[i];
        }
    } else still_s = 0;
    double ps[2];
    gyro_player_space(w, up, ps);
    double speed = hypot(ps[0], ps[1]) * RAD_DEG;
    double tighten = speed < TIGHTEN_DEG_S ? speed / TIGHTEN_DEG_S : 1;
    if (last_take && now - last_take > STALE_NS) pending[0] = pending[1] = 0;
    else
        for (int i = 0; i < 2; i++) pending[i] += ps[i] * RAD_DEG * tighten * dt;
    plat_mutex_unlock(&lock);
}

int gyro_mouse_take(double deg[2]) {
    settings();
    uint64_t now = plat_now_ns();
    plat_mutex_lock(&lock);
    int fresh = last_take && now - last_take <= STALE_NS;
    last_take = now;
    deg[0] = fresh ? pending[0] * sens : 0;
    deg[1] = fresh ? pending[1] * sens : 0;
    pending[0] = pending[1] = 0;
    plat_mutex_unlock(&lock);
    return deg[0] != 0 || deg[1] != 0;
}
