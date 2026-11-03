#include "logic/sysid/step_test.h"

#include <math.h>

static uint32_t MsToTicks(uint32_t ms, float dt_s) {
    uint32_t t = (uint32_t)((float)ms / (dt_s * 1000.0f) + 0.5f);
    return (t == 0u) ? 1u : t;
}

static void EnterSeg(StepTest *st, StepSegment seg) {
    st->seg = (uint8_t)seg;
    st->seg_ticks = 0u;
    st->still_ticks = 0u;
}

// no_reverse の区間3: 止まった状態が続いたら true。
// 機体(ジャイロ・並進の速度)と、左右の車輪(エンコーダ)の両方が止まっていること。
// 車輪は「SYSID_STOP_COUNTS カウント/tick 以下」。速さは カウント × MM_PER_PULSE / dt なので、
// 0.5 カウントぶんの余裕を足した速さより小さいかで見る(float の == を避ける)
static bool CheckStopped(StepTest *st, const StepMeas *m) {
    float wheel_th = ((float)SYSID_STOP_COUNTS + 0.5f) * MM_PER_PULSE / st->dt_s;
    bool body = st->cfg.rotate ? (fabsf(m->omega_dps) < SYSID_STOP_OMEGA_DPS)
                               : (fabsf(m->v_mm_s) < SYSID_STOP_V_MM_S);
    bool wheels = (fabsf(m->vl_mm_s) < wheel_th) && (fabsf(m->vr_mm_s) < wheel_th);
    bool still = body && wheels;
    st->still_ticks = still ? st->still_ticks + 1u : 0u;
    return st->still_ticks >= MsToTicks(SYSID_STOP_HOLD_MS, st->dt_s);
}

// k 番目のステップの電圧と向きにする
static void SetStep(StepTest *st, uint16_t k) {
    uint16_t level = st->cfg.alternate ? (uint16_t)(k / 2u) : k;
    bool flip = st->cfg.alternate && ((k % 2u) == 1u);
    st->step = k;
    st->cur_volt = st->cfg.volt + (float)level * st->cfg.v_inc;
    st->cur_sign = flip ? -st->cfg.sign : st->cfg.sign;
}

void StepTest_Start(StepTest *st, const StepConfig *cfg, float dt_s) {
    st->cfg = *cfg;
    uint16_t levels = (cfg->levels == 0u) ? 1u : cfg->levels;
    st->steps = cfg->alternate ? (uint16_t)(levels * 2u) : levels;
    SetStep(st, 0u);
    st->seg = (uint8_t)(cfg->skip_pre ? STEP_SEG_ACC : STEP_SEG_PRE);
    st->final_rest = false;
    st->done = false;
    st->abort = (uint8_t)STEP_ABORT_NONE;
    st->seg_ticks = 0u;
    st->still_ticks = 0u;
    st->dt_s = dt_s;
    st->v_cmd = 0.0f;
    st->v_left = 0.0f;
    st->v_right = 0.0f;
    st->moved_l = false;
    st->moved_r = false;
    st->move_v_l = 0.0f;
    st->move_v_r = 0.0f;
}

// 上限を超えたら中止の理由を入れて true。車輪の速さはいつも、並進は距離、旋回は角速度を見る
static bool CheckAbort(StepTest *st, const StepMeas *m) {
    if (fabsf(m->vl_mm_s) > SYSID_ABORT_WHEEL_MM_S || fabsf(m->vr_mm_s) > SYSID_ABORT_WHEEL_MM_S) {
        st->abort = (uint8_t)STEP_ABORT_WHEEL;
        return true;
    }
    if (st->cfg.rotate) {
        if (fabsf(m->omega_dps) > SYSID_ABORT_OMEGA_DPS) {
            st->abort = (uint8_t)STEP_ABORT_OMEGA;
            return true;
        }
    } else {
        float dist_max = (st->cfg.abort_dist_mm > 0.0f) ? st->cfg.abort_dist_mm : SYSID_ABORT_DIST_MM;
        if (!st->cfg.no_dist_abort && fabsf(0.5f * (m->dist_l_mm + m->dist_r_mm)) > dist_max) {
            st->abort = (uint8_t)STEP_ABORT_DIST;
            return true;
        }
        if (st->cfg.abort_v_mm_s > 0.0f && fabsf(m->v_mm_s) > st->cfg.abort_v_mm_s) {
            st->abort = (uint8_t)STEP_ABORT_SPEED;
            return true;
        }
        if (st->cfg.abort_angle_deg > 0.0f && fabsf(m->angle_deg) > st->cfg.abort_angle_deg) {
            st->abort = (uint8_t)STEP_ABORT_ANGLE;
            return true;
        }
    }
    return false;
}

// E0: 前の tick までにかけた電圧で、左右の車輪が動き出したかを見る(旋回では左は後ろへ動くので大きさで見る)
static void CheckMoved(StepTest *st, const StepMeas *m) {
    if (!st->moved_l && fabsf(m->dist_l_mm) >= SYSID_E0_MOVE_MM) {
        st->moved_l = true;
        st->move_v_l = fabsf(st->v_cmd);
    }
    if (!st->moved_r && fabsf(m->dist_r_mm) >= SYSID_E0_MOVE_MM) {
        st->moved_r = true;
        st->move_v_r = fabsf(st->v_cmd);
    }
}

void StepTest_Update(StepTest *st, const StepMeas *m) {
    if (st->done) {
        st->v_cmd = 0.0f;
        st->v_left = 0.0f;
        st->v_right = 0.0f;
        return;
    }

    // 中止を見るのは区間0〜2(電圧をかけている間)。区間4 は止まった後なので見ない(E1S では、ブレーキで曲がった角度で
    // 休みの間に中止にならないように)
    if (st->seg != STEP_SEG_BRAKE && st->seg != STEP_SEG_REST && st->seg != STEP_SEG_ABORT && CheckAbort(st, m)) {
        EnterSeg(st, STEP_SEG_ABORT);
    }

    // 区間の終わりを見る(この tick の電圧を決める前に)
    switch (st->seg) {
        case STEP_SEG_PRE:
            if (st->seg_ticks >= MsToTicks(SYSID_PRE_MS, st->dt_s)) EnterSeg(st, STEP_SEG_ACC);
            break;
        case STEP_SEG_ACC:
            if (st->cfg.type == STEP_TEST_RAMP) {
                CheckMoved(st, m);
                if ((st->moved_l && st->moved_r) || fabsf(st->v_cmd) >= SYSID_E0_V_MAX) {
                    EnterSeg(st, STEP_SEG_BRAKE);
                }
            } else if (st->seg_ticks >= MsToTicks(st->cfg.t_acc_ms, st->dt_s)) {
                EnterSeg(st, st->cfg.no_reverse ? STEP_SEG_BRAKE : STEP_SEG_REV);
            }
            break;
        case STEP_SEG_REV: {
            // 動いていた向きの速度(旋回は角速度)が 0 以下になったら止まったとする
            float x = st->cfg.rotate ? m->omega_dps : m->v_mm_s;
            if (st->cur_sign * x <= 0.0f) {
                EnterSeg(st, STEP_SEG_BRAKE);
            } else if (st->seg_ticks >= MsToTicks(SYSID_REVERSE_TIMEOUT_MS, st->dt_s)) {
                st->abort = (uint8_t)STEP_ABORT_REVERSE;
                EnterSeg(st, STEP_SEG_ABORT);
            }
            break;
        }
        case STEP_SEG_BRAKE: {
            // no_reverse なら止まるまで(止まらなければ中止)、そうでなければ決まった時間
            bool end;
            if (st->cfg.no_reverse) {
                end = CheckStopped(st, m);
                if (!end && st->seg_ticks >= MsToTicks(SYSID_STOP_TIMEOUT_MS, st->dt_s)) {
                    st->abort = (uint8_t)STEP_ABORT_NO_STOP;
                    EnterSeg(st, STEP_SEG_ABORT);
                    break;
                }
            } else {
                end = st->seg_ticks >= MsToTicks(SYSID_BRAKE_MS, st->dt_s);
            }
            if (end) {
                if (st->step + 1u < st->steps) { // 階段: 休んでから次のステップ
                    SetStep(st, (uint16_t)(st->step + 1u));
                    EnterSeg(st, STEP_SEG_REST);
                } else if (st->cfg.rest_after) { // 最後のステップの後も休んでから終わる
                    st->final_rest = true;
                    EnterSeg(st, STEP_SEG_REST);
                } else {
                    st->done = true;
                }
            }
            break;
        }
        case STEP_SEG_REST:
            if (st->seg_ticks >= MsToTicks(SYSID_REST_MS, st->dt_s)) {
                if (st->final_rest) {
                    st->done = true;
                } else {
                    EnterSeg(st, STEP_SEG_ACC);
                }
            }
            break;
        case STEP_SEG_ABORT:
        default:
            // 残りのステップは捨てる。no_reverse なら止まるまで待ってから終わる(終わるとドライバを無効にして空転になるため)
            if (st->cfg.no_reverse) {
                if (CheckStopped(st, m) || st->seg_ticks >= MsToTicks(SYSID_STOP_TIMEOUT_MS, st->dt_s)) st->done = true;
            } else if (st->seg_ticks >= MsToTicks(SYSID_BRAKE_MS, st->dt_s)) {
                st->done = true;
            }
            break;
    }

    float v = 0.0f; // この tick にかける電圧の大きさ
    if (!st->done) {
        if (st->seg == STEP_SEG_ACC) {
            v = (st->cfg.type == STEP_TEST_RAMP) ? SYSID_E0_RAMP_V_PER_S * (float)st->seg_ticks * st->dt_s
                                                 : st->cur_volt;
        } else if (st->seg == STEP_SEG_REV) {
            v = -st->cur_volt;
        }
    }

    st->v_cmd = st->cur_sign * v;
    if (st->cfg.rotate) {
        st->v_right = st->v_cmd;
        st->v_left = -st->v_cmd;
    } else {
        st->v_right = st->v_cmd;
        st->v_left = st->v_cmd;
    }
    st->seg_ticks++;
}
