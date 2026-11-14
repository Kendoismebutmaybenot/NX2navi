// Player space gyro through the mouse camera (gyro_mouse.c).
#pragma once
#include <stdint.h>

// 1 while NX2NAVI_GYRO_MODE is player, motion is on and the mouse camera's patch set runs (live).
int gyro_mouse_on(void);
// One sensor sample (input.c motion_main): acceleration in G, rate in rad/s, both in the Switch controller frame, and
// the fusion's up direction in that frame.
void gyro_mouse_feed(const double acc[3], const double gyro[3], const double up[3]);
// 1 while NX2NAVI_GYRO_SCOPE is aim: the turn counts only while Link aims (mouse_direct.c link_aiming).
int gyro_mouse_aim_only(void);
// The camera turn gathered since the last call, in degrees (right, up), times NX2NAVI_GYRO_SENS; 1 when nonzero.
int gyro_mouse_take(double deg[2]);
// The rate w (rad/s) in player space: out[0] the turn to the right, out[1] the turn up, rad/s.
void gyro_player_space(const double w[3], const double up[3], double out[2]);
