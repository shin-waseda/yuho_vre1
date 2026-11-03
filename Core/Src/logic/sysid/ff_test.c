#include "logic/sysid/ff_test.h"

#include <math.h>

#define DEG_TO_RAD (3.14159265f / 180.0f)

static uint32_t MsToTicks(uint32_t ms, float dt_s) {
    uint32_t t = (uint32_t)((float)ms / (dt_s * 1000.0f) + 0.5f);
    return (t == 0u) ? 1u : t;
}

static void EnterSeg(FfTest *ft, FfSegment seg) {
    ft->seg = (uint8_t)seg;
    ft->seg_ticks = 0u;
    ft->still_ticks = 0u;
}

// 止まった状態が SYSID_STOP_HOLD_MS 続いたら true(step_test の並進の判定と同じ)
static bool CheckStopped(FfTest *ft, const StepMeas *m) {
    float wheel_th = ((float)SYSID_STOP_COUNTS + 0.5f) * MM_PER_PULSE / ft->dt_s;
    bool still = (fabsf(m->v_mm_s) < SYSID_STOP_V_MM_S) && (fabsf(m->vl_mm_s) < wheel_th) &&
                 (fabsf(m->vr_mm_s) < wheel_th);
    ft->still_ticks = still ? ft->still_ticks + 1u : 0u;
    return ft->still_ticks >= MsToTicks(SYSID_STOP_HOLD_MS, ft->dt_s);
}

// 上限を超えたら中止の理由を入れて true。電圧は前の tick に出した値で見る
static bool CheckAbort(FfTest *ft, const StepMeas *m) {
    if (ft->cfg.abort_dist_mm > 0.0f && fabsf(0.5f * (m->dist_l_mm + m->dist_r_mm)) > ft->cfg.abort_dist_mm) {
        ft->abort = (uint8_t)STEP_ABORT_DIST;
        return true;
    }
    if (ft->cfg.abort_v_mm_s > 0.0f && fabsf(m->v_mm_s) > ft->cfg.abort_v_mm_s) {
        ft->abort = (uint8_t)STEP_ABORT_SPEED;
        return true;
    }
    if (ft->cfg.abort_back_mm_s > 0.0f && m->v_mm_s < -ft->cfg.abort_back_mm_s) {
        ft->abort = (uint8_t)STEP_ABORT_BACKWARD;
        return true;
    }
    if (ft->cfg.abort_angle_deg > 0.0f && fabsf(m->angle_deg) > ft->cfg.abort_angle_deg) {
        ft->abort = (uint8_t)STEP_ABORT_ANGLE;
        return true;
    }
    if (ft->cfg.abort_omega_dps > 0.0f && fabsf(m->omega_dps) > ft->cfg.abort_omega_dps) {
        ft->abort = (uint8_t)STEP_ABORT_OMEGA;
        return true;
    }
    if (ft->cfg.abort_volt_over && (fabsf(ft->v_left) > m->vbat_v || fabsf(ft->v_right) > m->vbat_v)) {
        ft->abort = (uint8_t)STEP_ABORT_VOLT;
        return true;
    }
    if (ft->cfg.abort_front_ir > 0.0f) {
        ft->front_ticks = (m->front_ir > ft->cfg.abort_front_ir) ? ft->front_ticks + 1u : 0u;
        if (ft->front_ticks >= MsToTicks(ft->cfg.abort_front_ms, ft->dt_s)) {
            ft->abort = (uint8_t)STEP_ABORT_OBSTACLE;
            return true;
        }
    }
    return false;
}

void FfTest_Start(FfTest *ft, const FfConfig *cfg, float dt_s) {
    ft->cfg = *cfg;
    ft->seg = (uint8_t)FF_SEG_PRE;
    ft->done = false;
    ft->abort = (uint8_t)STEP_ABORT_NONE;
    ft->seg_ticks = 0u;
    ft->still_ticks = 0u;
    ft->dt_s = dt_s;
    ft->v_ref = 0.0f;
    ft->a_ref = 0.0f;
    ft->v_cmd = 0.0f;
    ft->v_ff = 0.0f;
    ft->v_p = 0.0f;
    ft->v_i = 0.0f;
    ft->integ = 0.0f;
    ft->v_d = 0.0f;
    ft->theta_rad = 0.0f;
    ft->vd_sat = false;
    ft->theta_offset_deg = 0.0f;
    ft->turn_start_deg = 0.0f;
    ft->vd_sum = 0.0f;
    ft->vd_count = 0u;
    ft->v_d_meas = 0.0f;
    ft->v_d0 = 0.0f;
    ft->dv_out = 0.0f;
    ft->turn_idx = 0u;
    ft->front_ticks = 0u;
    ft->v_left = 0.0f;
    ft->v_right = 0.0f;
}

// E1W の段の数(1 段なら 1)
static uint8_t TurnSteps(const FfTest *ft) {
    return (ft->cfg.turn_seq_n >= 2u) ? ft->cfg.turn_seq_n : 1u;
}

uint8_t FfTest_LogSeg(const FfTest *ft) {
    if (!ft->cfg.turn_test) return ft->seg;
    uint8_t n = TurnSteps(ft);
    switch (ft->seg) {
        case FF_SEG_TURN: return (uint8_t)(3u + ft->turn_idx);
        case FF_SEG_DEC:  return (uint8_t)(3u + n);
        case FF_SEG_STOP: return (uint8_t)(4u + n);
        default:          return ft->seg; // 0 始まり、1 加速、2 一定、9 中止
    }
}

// 並進の PI。e = v_ref − v。積分は |ki·∫e dt| ≤ i_max で止める(止めている間は積分を進めない)
static void UpdatePi(FfTest *ft, float v_ref, const StepMeas *m) {
    float e = v_ref - m->v_mm_s;
    ft->v_p = ft->cfg.kp * e;
    if (ft->cfg.ki > 0.0f) {
        float lim = ft->cfg.i_max / ft->cfg.ki;
        ft->integ += e * ft->dt_s;
        if (ft->integ > lim) ft->integ = lim;
        if (ft->integ < -lim) ft->integ = -lim;
    }
    ft->v_i = ft->cfg.ki * ft->integ;
}

void FfTest_Update(FfTest *ft, const StepMeas *m) {
    ft->theta_rad = (m->angle_deg - ft->theta_offset_deg) * DEG_TO_RAD;
    ft->vd_sat = false;
    ft->v_d0 = 0.0f;
    ft->dv_out = 0.0f;
    if (ft->done) {
        ft->v_ref = 0.0f;
        ft->a_ref = 0.0f;
        ft->v_cmd = 0.0f;
        ft->v_ff = 0.0f;
        ft->v_p = 0.0f;
        ft->v_i = 0.0f;
        ft->v_d = 0.0f;
        ft->v_left = 0.0f;
        ft->v_right = 0.0f;
        return;
    }

    bool moving = (ft->seg == FF_SEG_ACC || ft->seg == FF_SEG_CRUISE || ft->seg == FF_SEG_TURN ||
                   ft->seg == FF_SEG_DEC);
    if (moving && CheckAbort(ft, m)) EnterSeg(ft, FF_SEG_ABORT);

    float a = ft->cfg.accel_mm_s2;
    float d = (ft->cfg.decel_mm_s2 > 0.0f) ? ft->cfg.decel_mm_s2 : a; // 減速度
    float t = (float)ft->seg_ticks * ft->dt_s; // 今の区間に入ってからの時間(この tick の前まで)

    // 区間の終わりを見る(この tick の電圧を決める前に)
    switch (ft->seg) {
        case FF_SEG_PRE:
            if (ft->seg_ticks >= MsToTicks(SYSID_PRE_MS, ft->dt_s)) EnterSeg(ft, FF_SEG_ACC);
            break;
        case FF_SEG_ACC:
            if (a * t >= ft->cfg.v_max_mm_s) {
                if (ft->cfg.cruise_theta_reset) { // FB の θ を 0 に入れ直す(ログの angle はそのまま)
                    ft->theta_offset_deg = m->angle_deg;
                    ft->theta_rad = 0.0f;
                }
                EnterSeg(ft, FF_SEG_CRUISE);
            }
            break;
        case FF_SEG_CRUISE:
            if (ft->seg_ticks >= MsToTicks(ft->cfg.cruise_ms, ft->dt_s)) {
                if (ft->cfg.turn_test) { // 一定の区間の最後の V_d の平均を記録して、ステップへ
                    ft->v_d_meas = (ft->vd_count > 0u) ? ft->vd_sum / (float)ft->vd_count : 0.0f;
                    ft->turn_start_deg = m->angle_deg;
                    EnterSeg(ft, FF_SEG_TURN);
                } else {
                    EnterSeg(ft, FF_SEG_DEC);
                }
            }
            break;
        case FF_SEG_TURN:
            if (ft->turn_idx + 1u < TurnSteps(ft) && ft->seg_ticks >= MsToTicks(ft->cfg.turn_ms, ft->dt_s)) {
                ft->turn_idx++; // E1W: 次の段へ(seg 列もこの tick から変わる)
                ft->seg_ticks = 0u;
            } else if (ft->seg_ticks >= MsToTicks(ft->cfg.turn_ms, ft->dt_s) ||
                       (ft->cfg.turn_deg > 0.0f && fabsf(m->angle_deg - ft->turn_start_deg) > ft->cfg.turn_deg)) {
                ft->theta_offset_deg = m->angle_deg; // FB の θ を 0 に入れ直す(ログの angle はそのまま)
                ft->theta_rad = 0.0f;
                EnterSeg(ft, FF_SEG_DEC);
            }
            break;
        case FF_SEG_DEC:
            if (ft->cfg.brake_test) { // 一定の電圧のブレーキ: 遅くなったら 0V へ。終わらなければ中止
                if (m->v_mm_s < ft->cfg.brake_end_mm_s) {
                    EnterSeg(ft, FF_SEG_STOP);
                } else if (ft->seg_ticks >= MsToTicks(SYSID_STOP_TIMEOUT_MS, ft->dt_s)) {
                    ft->abort = (uint8_t)STEP_ABORT_NO_STOP;
                    EnterSeg(ft, FF_SEG_ABORT);
                }
            } else if (ft->cfg.v_max_mm_s - d * t <= 0.0f) {
                EnterSeg(ft, FF_SEG_STOP);
            }
            break;
        case FF_SEG_STOP:
            if (CheckStopped(ft, m)) {
                ft->done = true;
            } else if (ft->seg_ticks >= MsToTicks(SYSID_STOP_TIMEOUT_MS, ft->dt_s)) {
                ft->abort = (uint8_t)STEP_ABORT_NO_STOP;
                ft->done = true;
            }
            break;
        case FF_SEG_ABORT:
        default:
            if (CheckStopped(ft, m) || ft->seg_ticks >= MsToTicks(SYSID_STOP_TIMEOUT_MS, ft->dt_s)) ft->done = true;
            break;
    }

    // この tick の目標(区間が変わったら新しい区間の始まりから)
    float t1 = (float)(ft->seg_ticks + 1u) * ft->dt_s;
    float v_ref = 0.0f;
    float a_ref = 0.0f;
    if (!ft->done) {
        switch (ft->seg) {
            case FF_SEG_ACC:
                v_ref = fminf(a * t1, ft->cfg.v_max_mm_s);
                a_ref = a;
                break;
            case FF_SEG_CRUISE:
            case FF_SEG_TURN:
                v_ref = ft->cfg.v_max_mm_s;
                break;
            case FF_SEG_DEC:
                if (!ft->cfg.brake_test) { // ブレーキの試験では v_ref・a_ref は 0(電圧は brake_v)
                    v_ref = fmaxf(ft->cfg.v_max_mm_s - d * t1, 0.0f);
                    a_ref = -d;
                }
                break;
            default:
                break;
        }
    }
    ft->v_ref = v_ref;
    ft->a_ref = (v_ref > 0.0f) ? a_ref : 0.0f;

    bool braking = !ft->done && ft->cfg.brake_test && ft->seg == FF_SEG_DEC;
    if (v_ref > 0.0f || braking) {
        float v;
        if (braking) {
            ft->v_ff = ft->cfg.brake_v;
            ft->v_p = 0.0f;
            ft->v_i = 0.0f;
        } else {
            ft->v_ff = ft->cfg.kv * v_ref + ft->cfg.v0 + ft->cfg.ka * a_ref;
            UpdatePi(ft, v_ref, m);
        }
        v = ft->v_ff + ft->v_p + ft->v_i;
        ft->v_cmd = v;
        float dv = ft->cfg.lr_c * v_ref + ft->cfg.lr_a * a_ref;
        float vd;
        if (ft->seg == FF_SEG_TURN) { // ステップ: FB をやめ、V_d0 ± ΔV(V_d0 はこの本の区間2 の最後の V_d の平均)
            ft->v_d0 = ft->cfg.turn_no_vd0 ? 0.0f : ft->v_d_meas; // E1K0 は V_d0 を足さない
            ft->dv_out = (TurnSteps(ft) >= 2u) ? ft->cfg.turn_seq_dv[ft->turn_idx] : ft->cfg.turn_dv;
            vd = ft->v_d0 + ft->dv_out;
        } else {
            float omega_rad_s = m->omega_dps * DEG_TO_RAD;
            vd = -ft->cfg.k_theta * ft->theta_rad - ft->cfg.k_omega * omega_rad_s;
            if (vd > ft->cfg.vd_max) {
                vd = ft->cfg.vd_max;
                ft->vd_sat = true;
            } else if (vd < -ft->cfg.vd_max) {
                vd = -ft->cfg.vd_max;
                ft->vd_sat = true;
            }
            // E1T: 一定の区間の最後の vd_meas_ms の V_d を足し込む
            if (ft->cfg.turn_test && ft->seg == FF_SEG_CRUISE) {
                uint32_t n_cruise = MsToTicks(ft->cfg.cruise_ms, ft->dt_s);
                uint32_t n_meas = MsToTicks(ft->cfg.vd_meas_ms, ft->dt_s);
                if (ft->seg_ticks + n_meas >= n_cruise) {
                    ft->vd_sum += vd;
                    ft->vd_count++;
                }
            }
        }
        ft->v_d = vd;
        ft->v_left = v + dv - vd;
        ft->v_right = v - dv + vd;
    } else {
        ft->v_cmd = 0.0f;
        ft->v_ff = 0.0f;
        ft->v_p = 0.0f;
        ft->v_i = 0.0f;
        ft->v_d = 0.0f;
        ft->v_left = 0.0f;
        ft->v_right = 0.0f;
    }
    ft->seg_ticks++;
}
