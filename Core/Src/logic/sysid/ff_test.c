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
    if (ft->cfg.abort_track_deg > 0.0f && (ft->cfg.fb_track || ft->seg != FF_SEG_DEC) &&
        fabsf((m->angle_deg - ft->theta_offset_deg) - ft->theta_ref / DEG_TO_RAD) > ft->cfg.abort_track_deg) {
        ft->abort = (uint8_t)STEP_ABORT_ANGLE; // E1C: θ − θ_ref(区間5 は θ を入れ直すので見ない。E1F は入れ直さないので見る)
        return true;
    }
    if (ft->cfg.abort_fb_sat_ms > 0u && ft->fb_sat_ticks >= MsToTicks(ft->cfg.abort_fb_sat_ms, ft->dt_s)) {
        ft->abort = (uint8_t)STEP_ABORT_FB_SAT; // E1F: FB を上限で切った状態が続いた
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
    ft->v_s_ff = 0.0f;
    ft->turn_phase = 0u;
    ft->omega_ref = 0.0f;
    ft->omega_dot_ref = 0.0f;
    ft->theta_ref = 0.0f;
    ft->f_ff = 0.0f;
    ft->v_d_ff = 0.0f;
    ft->fb_w = 0.0f;
    ft->fb_th = 0.0f;
    ft->fb_sat_ticks = 0u;
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
    uint8_t n = TurnSteps(ft) + (ft->cfg.turn_ff ? 1u : 0u); // E1C はターンの後の区間の分だけ後ろへ
    switch (ft->seg) {
        case FF_SEG_TURN: return (uint8_t)(3u + ft->turn_idx);
        case FF_SEG_POST: return 4u;
        case FF_SEG_DEC:  return (uint8_t)(3u + n);
        case FF_SEG_STOP: return (uint8_t)(4u + n);
        default:          return ft->seg; // 0 始まり、1 加速、2 一定、9 中止
    }
}

// E1C: κ₀(v)[1/m]。表の間は直線でつなぎ、外は端の値
static float Kappa0(const FfConfig *c, float v_mm_s) {
    if (c->k0_n == 0u) return 1.0f;
    if (v_mm_s <= c->k0_v[0]) return c->k0[0];
    for (uint8_t i = 1u; i < c->k0_n; i++) {
        if (v_mm_s <= c->k0_v[i]) {
            float r = (v_mm_s - c->k0_v[i - 1u]) / (c->k0_v[i] - c->k0_v[i - 1u]);
            return c->k0[i - 1u] + r * (c->k0[i] - c->k0[i - 1u]);
        }
    }
    return c->k0[c->k0_n - 1u];
}

// E1C: この tick の ω_ref・ω̇_ref を台形で進め、θ_ref に足す。ω_ref が 0 に戻ったら turn_phase = 3
// (大きさで計算して turn_sign を掛ける。角減速に入るのは、残りの角度が ω²/(2α) + ω·Δt/2 以下になったとき)
static void TurnTrapezoid(FfTest *ft) {
    float dt = ft->dt_s;
    float w_max = ft->cfg.turn_omega_dps * DEG_TO_RAD;
    float alpha = ft->cfg.turn_alpha_dps2 * DEG_TO_RAD;
    float target = ft->cfg.turn_angle_deg * DEG_TO_RAD;
    float w = fabsf(ft->omega_ref);
    float th = fabsf(ft->theta_ref);
    if (ft->turn_phase < 2u && target - th <= w * w / (2.0f * alpha) + 0.5f * w * dt) ft->turn_phase = 2u;
    float w_new = w;
    if (ft->turn_phase == 0u) {
        w_new = fminf(w + alpha * dt, w_max);
        if (w_new >= w_max) ft->turn_phase = 1u;
    } else if (ft->turn_phase == 2u) {
        w_new = fmaxf(w - alpha * dt, 0.0f);
        if (w_new <= 0.0f) ft->turn_phase = 3u;
    }
    float s = ft->cfg.turn_sign;
    ft->omega_dot_ref = s * (w_new - w) / dt;
    ft->omega_ref = s * w_new;
    ft->theta_ref += ft->omega_ref * dt;
}

// E1C: F_ff を1 tick 進める(h は今の ω_ref から。ω_ref = 0 なら h = 0 で 0 に近づく)
static void UpdateFff(FfTest *ft) {
    float v_m_s = ft->v_ref / 1000.0f;
    float h = 0.0f;
    if (v_m_s > 0.0f) {
        float kappa = ft->omega_ref / v_m_s;
        float a = (kappa >= 0.0f) ? ft->cfg.a_pos : ft->cfg.a_neg;
        h = a * asinhf(kappa / Kappa0(&ft->cfg, ft->v_ref));
    }
    float g = fminf(ft->v_ref * ft->dt_s / ft->cfg.sigma_mm, 1.0f);
    ft->f_ff += g * (h - ft->f_ff);
}

// 旋回の FB: k_omega·(ω_ref − ω) + k_theta·(θ_ref − θ) を ±vd_max で切る(切ったら vd_sat)。
// 目標を追うのは E1F(fb_track)だけで、ほかの試験は ω_ref = θ_ref = 0(−k_theta·θ − k_omega·ω)
static float YawFb(FfTest *ft, const StepMeas *m) {
    float w_ref = ft->cfg.fb_track ? ft->omega_ref : 0.0f;
    float th_ref = ft->cfg.fb_track ? ft->theta_ref : 0.0f;
    ft->fb_w = ft->cfg.k_omega * (w_ref - m->omega_dps * DEG_TO_RAD);
    ft->fb_th = ft->cfg.k_theta * (th_ref - ft->theta_rad);
    float fb = ft->fb_w + ft->fb_th;
    if (fb > ft->cfg.vd_max) {
        fb = ft->cfg.vd_max;
        ft->vd_sat = true;
    } else if (fb < -ft->cfg.vd_max) {
        fb = -ft->cfg.vd_max;
        ft->vd_sat = true;
    }
    return fb;
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
    ft->v_d_ff = 0.0f;
    ft->fb_w = 0.0f;
    ft->fb_th = 0.0f;
    if (ft->done) {
        ft->v_ref = 0.0f;
        ft->a_ref = 0.0f;
        ft->v_cmd = 0.0f;
        ft->v_ff = 0.0f;
        ft->v_s_ff = 0.0f;
        ft->omega_ref = 0.0f;
        ft->omega_dot_ref = 0.0f;
        ft->v_p = 0.0f;
        ft->v_i = 0.0f;
        ft->v_d = 0.0f;
        ft->v_left = 0.0f;
        ft->v_right = 0.0f;
        return;
    }

    bool moving = (ft->seg == FF_SEG_ACC || ft->seg == FF_SEG_CRUISE || ft->seg == FF_SEG_TURN ||
                   ft->seg == FF_SEG_POST || ft->seg == FF_SEG_DEC);
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
                    if (ft->cfg.turn_ff) { // E1C: θ を 0 に入れ直し(ログの angle はそのまま)、FF の状態を 0 から
                        ft->theta_offset_deg = m->angle_deg;
                        ft->theta_rad = 0.0f;
                        ft->turn_phase = 0u;
                        ft->theta_ref = 0.0f;
                        ft->f_ff = 0.0f;
                    }
                    EnterSeg(ft, FF_SEG_TURN);
                } else {
                    EnterSeg(ft, FF_SEG_DEC);
                }
            }
            break;
        case FF_SEG_TURN:
            if (ft->cfg.turn_ff) { // E1C: ω_ref の台形が 0 に戻ったら、ターンの後の直進へ
                if (ft->turn_phase >= 3u) EnterSeg(ft, FF_SEG_POST);
            } else if (ft->turn_idx + 1u < TurnSteps(ft) && ft->seg_ticks >= MsToTicks(ft->cfg.turn_ms, ft->dt_s)) {
                ft->turn_idx++; // E1W: 次の段へ(seg 列もこの tick から変わる)
                ft->seg_ticks = 0u;
            } else if (ft->seg_ticks >= MsToTicks(ft->cfg.turn_ms, ft->dt_s) ||
                       (ft->cfg.turn_deg > 0.0f && fabsf(m->angle_deg - ft->turn_start_deg) > ft->cfg.turn_deg)) {
                ft->theta_offset_deg = m->angle_deg; // FB の θ を 0 に入れ直す(ログの angle はそのまま)
                ft->theta_rad = 0.0f;
                EnterSeg(ft, FF_SEG_DEC);
            }
            break;
        case FF_SEG_POST:
            if (ft->seg_ticks >= MsToTicks(ft->cfg.post_ms, ft->dt_s)) {
                if (!ft->cfg.fb_track) { // E1C: FB の θ を 0 に入れ直す(ログの angle はそのまま)。E1F は θ_ref を追い続ける
                    ft->theta_offset_deg = m->angle_deg;
                    ft->theta_rad = 0.0f;
                }
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
            case FF_SEG_POST:
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

    // E1C: ターン(と、その後の直進)の ω_ref・ω̇_ref・θ_ref と F_ff を進める
    bool ff_turn = ft->cfg.turn_ff && (ft->seg == FF_SEG_TURN || ft->seg == FF_SEG_POST);
    if (ff_turn && ft->seg == FF_SEG_TURN) {
        TurnTrapezoid(ft);
    } else {
        ft->omega_ref = 0.0f;
        ft->omega_dot_ref = 0.0f;
    }
    if (ff_turn) UpdateFff(ft);

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
        // E1C: 結合の項(曲がると遅くなる分)を足す。ほかの試験は ω_ref = 0 なので v_ff のまま
        ft->v_s_ff = ft->v_ff + ft->cfg.k_couple * (v_ref / 1000.0f) * ft->omega_ref * ft->omega_ref;
        v = ft->v_s_ff + ft->v_p + ft->v_i;
        ft->v_cmd = v;
        float dv = ft->cfg.lr_c * v_ref + ft->cfg.lr_a * a_ref;
        float vd;
        if (ff_turn) { // E1C: FB をやめ、V_d = V_d0 + b_ω ω̇_ref + F_ff(V_d0 はこの本の区間2 の最後の V_d の平均)
            ft->v_d0 = ft->v_d_meas;
            ft->dv_out = ft->cfg.b_omega * ft->omega_dot_ref + ft->f_ff;
            vd = ft->v_d0 + ft->dv_out;
            ft->v_d_ff = vd;
            if (ft->cfg.fb_track) vd += YawFb(ft, m); // E1F: ターンの間も FB を足す
        } else if (ft->seg == FF_SEG_TURN) { // ステップ: FB をやめ、V_d0 ± ΔV(V_d0 はこの本の区間2 の最後の V_d の平均)
            ft->v_d0 = ft->cfg.turn_no_vd0 ? 0.0f : ft->v_d_meas; // E1K0 は V_d0 を足さない
            ft->dv_out = (TurnSteps(ft) >= 2u) ? ft->cfg.turn_seq_dv[ft->turn_idx] : ft->cfg.turn_dv;
            vd = ft->v_d0 + ft->dv_out;
        } else {
            vd = YawFb(ft, m);
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
        if (ft->cfg.fb_track) ft->fb_sat_ticks = ft->vd_sat ? ft->fb_sat_ticks + 1u : 0u;
    } else {
        ft->v_cmd = 0.0f;
        ft->v_ff = 0.0f;
        ft->v_s_ff = 0.0f;
        ft->v_p = 0.0f;
        ft->v_i = 0.0f;
        ft->v_d = 0.0f;
        ft->v_left = 0.0f;
        ft->v_right = 0.0f;
    }
    ft->seg_ticks++;
}
