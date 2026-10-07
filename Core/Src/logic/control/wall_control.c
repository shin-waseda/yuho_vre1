#include "logic/control/wall_control.h"

#include <stddef.h>

void WallControl_Reset(WallControl *wc) {
    wc->started = false;
}

static void SideStart(WallSideState *s, uint16_t value, float dist_mm) {
    s->last_value = value;
    s->last_dist_mm = dist_mm;
    s->hold_until_mm = dist_mm; // 始めはすぐ使ってよい
}

// WALL_EDGE_STEP_MM 進むごとに値を比べ、急に変わっていたら、しばらく使わないようにする。
// その側の壁を今使ってよいかを返す。
static bool SideUsable(WallSideState *s, uint16_t value, uint16_t threshold, float dist_mm) {
    if (dist_mm - s->last_dist_mm >= WALL_EDGE_STEP_MM) {
        int32_t diff = (int32_t)value - (int32_t)s->last_value;
        if (diff > WALL_EDGE_DIFF || diff < -WALL_EDGE_DIFF) {
            s->hold_until_mm = dist_mm + WALL_EDGE_HOLD_MM;
        }
        s->last_value = value;
        s->last_dist_mm = dist_mm;
    }
    return value > threshold && dist_mm >= s->hold_until_mm;
}

float WallControl_Update(WallControl *wc, WallSensorValues v, float dist_mm,
                         bool *used_left, bool *used_right) {
    if (!wc->started) {
        SideStart(&wc->left, v.l, dist_mm);
        SideStart(&wc->right, v.r, dist_mm);
        wc->started = true;
    }

    bool use_l = SideUsable(&wc->left, v.l, WALL_TH_L, dist_mm);
    bool use_r = SideUsable(&wc->right, v.r, WALL_TH_R, dist_mm);
    if (used_left != NULL) *used_left = use_l;
    if (used_right != NULL) *used_right = use_r;

    float err_l = (float)v.l - (float)WALL_REF_L; // 正: 左の壁に近い
    float err_r = (float)v.r - (float)WALL_REF_R; // 正: 右の壁に近い
    float err;
    if (use_l && use_r) {
        err = err_l - err_r;
    } else if (use_l) {
        err = 2.0f * err_l;
    } else if (use_r) {
        err = -2.0f * err_r;
    } else {
        return 0.0f;
    }

    // 左に寄っている(err > 0)なら右へ(時計回り = 負)
    float corr = -WALL_KP * err;
    if (corr > WALL_CORR_LIMIT_DPS) corr = WALL_CORR_LIMIT_DPS;
    if (corr < -WALL_CORR_LIMIT_DPS) corr = -WALL_CORR_LIMIT_DPS;
    return corr;
}
