#include "app/sysid_mode.h"

#include <math.h>
#include <stdio.h>
#include "main.h"
#include "app/mode_ui.h"
#include "app/sysid_loop.h"
#include "app/failsafe.h"
#include "app/logger.h"
#include "interface/led.h"
#include "interface/sdcard.h"

#define COUNT_OF(a) ((uint8_t)(sizeof(a) / sizeof((a)[0])))

static const float kSelectMv[] = SYSID_SELECT_MV;
static const float kSelectTaccMs[] = SYSID_SELECT_TACC_MS;
static const float kSelectRepeat[] = SYSID_SELECT_REPEAT;
static const float kStairMv[] = SYSID_STAIR_MV;
static const float kE1sMv[] = SYSID_E1S_MV;
static const float kE1vVmax[] = SYSID_E1V_VMAX_MM_S;
static const float kE1vAccel[] = SYSID_E1V_ACCEL_MM_S2;
static const float kE1bV0[] = SYSID_E1B_V0_MM_S;
static const float kE1bBrakeV[] = SYSID_E1B_BRAKE_V;
_Static_assert(COUNT_OF(kE1bV0) == COUNT_OF(kE1bBrakeV), "SYSID_E1B_V0_MM_S and SYSID_E1B_BRAKE_V must pair up");
// E1T の組(0 = B、1 = A、2 = C、3 = D、4 = E)ごとの速さ・加速度・直進の時間・V_d0 を測る時間・
// 区間3 の長さと角度の上限・中止の角度と ΔV の表
static const float kE1tSetSpeed[] = SYSID_E1T_SET_SPEED_MM_S;
static const float kE1tSetAccel[] = SYSID_E1T_SET_ACCEL_MM_S2;
static const uint32_t kE1tSetStraightMs[] = SYSID_E1T_SET_STRAIGHT_MS;
static const uint32_t kE1tSetMeasMs[] = SYSID_E1T_SET_MEAS_MS;
static const uint32_t kE1tSetTurnMs[] = SYSID_E1T_SET_TURN_MS;
static const float kE1tSetTurnDeg[] = SYSID_E1T_SET_TURN_DEG;
static const float kE1tSetAbortAngle[] = SYSID_E1T_SET_ABORT_ANGLE_DEG;
static const float kE1tDvB[] = SYSID_E1T_DV_B_V;
static const float kE1tDvA[] = SYSID_E1T_DV_A_V;
static const float kE1tDvC[] = SYSID_E1T_DV_C_V;
static const float kE1tDvD[] = SYSID_E1T_DV_D_V;
static const float kE1tDvE[] = SYSID_E1T_DV_E_V;
_Static_assert(COUNT_OF(kE1tSetSpeed) == 5u && COUNT_OF(kE1tSetAccel) == 5u && COUNT_OF(kE1tSetStraightMs) == 5u &&
                   COUNT_OF(kE1tSetMeasMs) == 5u && COUNT_OF(kE1tSetTurnMs) == 5u && COUNT_OF(kE1tSetTurnDeg) == 5u &&
                   COUNT_OF(kE1tSetAbortAngle) == 5u,
               "E1T has 5 sets (B, A, C, D, E)");
// E1K0 の速さごとの加速度・区間3(V_d = 0)の長さ(step_no で引く)
static const float kE1k0Speed[] = SYSID_E1K0_SPEED_MM_S;
static const float kE1k0Accel[] = SYSID_E1K0_ACCEL_MM_S2;
static const uint32_t kE1k0FreeMs[] = SYSID_E1K0_FREE_MS;
_Static_assert(COUNT_OF(kE1k0Accel) == COUNT_OF(kE1k0Speed) && COUNT_OF(kE1k0FreeMs) == COUNT_OF(kE1k0Speed),
               "SYSID_E1K0_* tables must pair up");
static const float kE1k0Vd[] = SYSID_E1K0_VD_V; // E1K0 の V_d を振る組: 区間3 の V_d(step_no で引く)
// E1W の速さ(SET 1・2 で取る順)ごとの加速度・直進・V_d0 を測る時間・区間の長さ・A の数と、A の表。
// step_no − 1 = 速さの番号 × A の表の長さ + A の番号
static const float kE1wSpeed[] = SYSID_E1W_SPEED_MM_S;
static const float kE1wAccel[] = SYSID_E1W_ACCEL_MM_S2;
static const uint32_t kE1wCruiseMs[] = SYSID_E1W_CRUISE_MS;
static const uint32_t kE1wMeasMs[] = SYSID_E1W_MEAS_MS;
static const uint32_t kE1wStepMs[] = SYSID_E1W_STEP_MS;
static const uint8_t kE1wNA[] = SYSID_E1W_N_A;
static const float kE1wA[] = SYSID_E1W_A_V;
_Static_assert(COUNT_OF(kE1wAccel) == COUNT_OF(kE1wSpeed) && COUNT_OF(kE1wCruiseMs) == COUNT_OF(kE1wSpeed) &&
                   COUNT_OF(kE1wMeasMs) == COUNT_OF(kE1wSpeed) && COUNT_OF(kE1wStepMs) == COUNT_OF(kE1wSpeed) &&
                   COUNT_OF(kE1wNA) == COUNT_OF(kE1wSpeed),
               "SYSID_E1W_* tables must pair up");
_Static_assert(COUNT_OF(kE1wSpeed) == 2u * SYSID_E1W_SET_SIZE, "SET 1 and SET 2 take SYSID_E1W_SET_SIZE speeds each");
_Static_assert(COUNT_OF(kE1vVmax) == 6u && COUNT_OF(kE1vAccel) == 6u, "fix the E1V run sets below");
// E1V で走る台形(表の何本目か。0 始まり)。走る前に選ぶ(1 全部、2 時間がないときに先に取る 1・3・4・6、3 残りの 2・5)
static const uint8_t kE1vSetAll[] = { 0u, 1u, 2u, 3u, 4u, 5u };
static const uint8_t kE1vSetFirst[] = { 0u, 2u, 3u, 5u };
static const uint8_t kE1vSetRest[] = { 1u, 4u };

// 1本の条件
typedef struct {
    float volt; // [V](E0 では使わない。E1V では v_max[mm/s])
    float sign; // +1: 前進・左回り、-1: 後退・右回り
    uint16_t step_no; // E1S: 何番目の電圧か、E1V: 台形の表の何本目か(1 始まり。ログの step 列)。ほかは 0
} RunPlan;

#define SYSID_MAX_RUNS (2u * 5u * 5u) // 電圧 5 つ × 回数 5 × 往復
_Static_assert(COUNT_OF(kSelectMv) <= 5u && COUNT_OF(kSelectRepeat) <= 5u, "enlarge SYSID_MAX_RUNS");
_Static_assert(COUNT_OF(kE1sMv) <= SYSID_MAX_RUNS, "enlarge SYSID_MAX_RUNS");
_Static_assert(COUNT_OF(kE1wSpeed) * COUNT_OF(kE1wA) * SYSID_E1W_REPEAT <= SYSID_MAX_RUNS, "enlarge SYSID_MAX_RUNS");
static RunPlan s_plan[SYSID_MAX_RUNS];

// 選んだ設定
static SysidTest s_test;
static bool s_rotate;
static uint32_t s_t_acc_ms;
static uint8_t s_stair_levels = 1u; // E2S: 電圧の段の数
static float s_stair_to_v = 0.0f;   // E2S・E1S: 最後の段の電圧(ファイル名・表示用)
static float s_from_v = 0.0f;       // E1S: 最初の電圧(ファイル名用)
static uint8_t s_e1v_set = 1u;      // E1V: 走る台形の組(1 全部、2 1・3・4・6、3 2・5)
static uint8_t s_e1t_set = 0u;      // E1T: 組(0 = B 1.0m/s、1 = A 0.5m/s、2 = C 1.5m/s、3 = D 1.0m/s 270°、4 = E 0.5m/s 270°)
static const float *s_e1t_dv = kE1tDvB; // E1T: 選んだ組の ΔV の表(step_no で引く)
static uint8_t s_e1t_dv_count = COUNT_OF(kE1tDvB);
static bool s_e1k0_vd = false;      // E1K0: V_d を振る組(SET 2)。false なら速さの組(SET 1)
static uint8_t s_e1w_set = 0u;      // E1W: 0 = SET 1(1.0 → 0.5 → 1.5m/s)、1 = SET 2(0.3 → 0.75 → 2.0m/s)、2 = 速さを1つ
static uint8_t s_e1w_speed = 0u;    // E1W: SET 3 で選んだ速さ(kE1wSpeed の番号)
static bool s_lifted = false;       // E2S・E1S: 機体を浮かせて空転させる(ファイル名を e2sl_・e1sl_ にする)

static char s_file_name[24]; // Logger_SetFileName はポインタを持つので static に置く

static const char *TestName(SysidTest test) {
    switch (test) {
        case SYSID_E0:  return "E0";
        case SYSID_E1:  return "E1";
        case SYSID_E2:  return "E2";
        case SYSID_E2S: return "E2S";
        case SYSID_E1S: return "E1S";
        case SYSID_E1V: return "E1V";
        case SYSID_E1B: return "E1B";
        case SYSID_E1T: return "E1T";
        case SYSID_E1K0: return "E1K0";
        case SYSID_E1W: return "E1W";
        default:        return "?";
    }
}

static const char *DirName(bool rotate, float sign) {
    if (rotate) return (sign > 0.0f) ? "ccw" : "cw";
    return (sign > 0.0f) ? "fwd" : "back";
}

static const char *AbortName(uint8_t abort) {
    switch (abort) {
        case STEP_ABORT_NONE:    return "ok";
        case STEP_ABORT_DIST:    return "ABORT(distance)";
        case STEP_ABORT_OMEGA:   return "ABORT(omega)";
        case STEP_ABORT_REVERSE: return "ABORT(did not stop)";
        case STEP_ABORT_NO_STOP: return "ABORT(did not stop at 0V)";
        case STEP_ABORT_WHEEL:   return "ABORT(wheel too fast)";
        case STEP_ABORT_SPEED:   return "ABORT(speed)";
        case STEP_ABORT_ANGLE:   return "ABORT(angle)";
        case STEP_ABORT_BACKWARD: return "ABORT(moved backward)";
        case STEP_ABORT_VOLT:    return "ABORT(|V_L| or |V_R| over battery)";
        case STEP_ABORT_OBSTACLE: return "ABORT(obstacle in front)";
        default:                 return "ABORT(?)";
    }
}

// 条件の並びを作り、本数を返す
static uint8_t BuildPlan(float from_mv, float to_mv, uint8_t repeat) {
    uint8_t n = 0;
    if (s_test == SYSID_E2S) { // 階段1つが1本(左右は階段の中で交互)
        for (uint8_t r = 0; r < repeat; r++) {
            s_plan[n++] = (RunPlan){ from_mv / 1000.0f, +1.0f, 0u };
        }
        return n;
    }
    if (s_test == SYSID_E1T) { // 選んだ組の速さで、ΔV の表の順に1本ずつ(ΔV は step_no で表を引く)
        for (uint8_t k = 0; k < s_e1t_dv_count; k++) {
            s_plan[n++] = (RunPlan){ kE1tSetSpeed[s_e1t_set], +1.0f, (uint16_t)(k + 1u) };
        }
        return n;
    }
    if (s_test == SYSID_E1W) { // 速さの順に、A の順に REPEAT 本ずつ(速さごとの値と A は step_no で表を引く)
        for (uint8_t s = 0; s < COUNT_OF(kE1wSpeed); s++) {
            bool use = (s_e1w_set == 2u) ? (s == s_e1w_speed) : (s / SYSID_E1W_SET_SIZE == s_e1w_set);
            if (!use) continue;
            for (uint8_t a = 0; a < kE1wNA[s] && a < COUNT_OF(kE1wA); a++) {
                for (uint8_t r = 0; r < SYSID_E1W_REPEAT; r++) {
                    s_plan[n++] = (RunPlan){ kE1wSpeed[s], +1.0f, (uint16_t)(s * COUNT_OF(kE1wA) + a + 1u) };
                }
            }
        }
        return n;
    }
    if (s_test == SYSID_E1K0 && s_e1k0_vd) { // V_d の表の順に1本ずつ(V_d は step_no で表を引く)
        for (uint8_t k = 0; k < COUNT_OF(kE1k0Vd); k++) {
            s_plan[n++] = (RunPlan){ SYSID_E1K0_VD_SPEED_MM_S, +1.0f, (uint16_t)(k + 1u) };
        }
        return n;
    }
    if (s_test == SYSID_E1K0) { // 速さの表を ROUNDS 周(速さ・加速度・区間3 の長さは step_no で表を引く)
        for (uint8_t r = 0; r < SYSID_E1K0_ROUNDS; r++) {
            for (uint8_t k = 0; k < COUNT_OF(kE1k0Speed); k++) {
                s_plan[n++] = (RunPlan){ kE1k0Speed[k], +1.0f, (uint16_t)(k + 1u) };
            }
        }
        return n;
    }
    if (s_test == SYSID_E1B) { // 表の順に1本ずつ(v0 と V_brake は step_no で表を引く)
        for (uint8_t k = 0; k < COUNT_OF(kE1bV0); k++) {
            s_plan[n++] = (RunPlan){ kE1bV0[k], +1.0f, (uint16_t)(k + 1u) };
        }
        return n;
    }
    if (s_test == SYSID_E1V) { // 選んだ組の台形を1本ずつ
        const uint8_t *set = kE1vSetAll;
        uint8_t count = COUNT_OF(kE1vSetAll);
        if (s_e1v_set == 2u) {
            set = kE1vSetFirst;
            count = COUNT_OF(kE1vSetFirst);
        } else if (s_e1v_set == 3u) {
            set = kE1vSetRest;
            count = COUNT_OF(kE1vSetRest);
        }
        for (uint8_t k = 0; k < count; k++) {
            s_plan[n++] = (RunPlan){ kE1vVmax[set[k]], +1.0f, (uint16_t)(set[k] + 1u) };
        }
        return n;
    }
    if (s_test == SYSID_E1S && s_lifted) { // 浮かせたら階段1つ(FROM〜TO)が1本
        s_plan[n++] = (RunPlan){ from_mv / 1000.0f, +1.0f, 1u };
        return n;
    }
    if (s_test == SYSID_E1S) { // 電圧ごとに前進を1本
        for (uint8_t k = 0; k < COUNT_OF(kE1sMv); k++) {
            float mv = kE1sMv[k];
            if (mv < from_mv || mv > to_mv) continue;
            s_plan[n] = (RunPlan){ mv / 1000.0f, +1.0f, (uint16_t)(n + 1u) };
            n++;
        }
        return n;
    }
    if (s_test == SYSID_E0) {
        for (uint8_t r = 0; r < repeat; r++) {
            s_plan[n++] = (RunPlan){ 0.0f, +1.0f, 0u };
            s_plan[n++] = (RunPlan){ 0.0f, -1.0f, 0u };
        }
        return n;
    }
    for (uint8_t k = 0; k < COUNT_OF(kSelectMv); k++) {
        float mv = kSelectMv[k];
        if (mv < from_mv || mv > to_mv) continue;
        for (uint8_t r = 0; r < repeat; r++) {
            s_plan[n++] = (RunPlan){ mv / 1000.0f, +1.0f, 0u };
            s_plan[n++] = (RunPlan){ mv / 1000.0f, -1.0f, 0u };
        }
    }
    return n;
}

static uint8_t IndexOf(const float *values, uint8_t count, float v) {
    for (uint8_t k = 0; k < count; k++) {
        if (values[k] == v) return k;
    }
    return 0u;
}

// 走る前に条件を選ぶ。本数を返す
static uint8_t SelectSettings(void) {
    float from_mv = 0.0f, to_mv = 0.0f;
    if (s_test == SYSID_E0) {
        static const float kKinds[] = { 1.0f, 2.0f };
        s_rotate = (ModeUI_SelectValue("E0 KIND(1 translation, 2 pivot)", "", kKinds, COUNT_OF(kKinds), 1.0f) == 2.0f);
        s_t_acc_ms = 0u;
    } else if (s_test == SYSID_E2S) {
        static const float kPlaces[] = { 1.0f, 2.0f };
        s_lifted = (ModeUI_SelectValue("PLACE(1 floor, 2 lifted)", "", kPlaces, COUNT_OF(kPlaces), 1.0f) == 2.0f);
        s_rotate = true;
        from_mv = ModeUI_SelectValue("VOLT FROM", "mV", kStairMv, COUNT_OF(kStairMv), kStairMv[0]);
        uint8_t from_i = IndexOf(kStairMv, COUNT_OF(kStairMv), from_mv);
        to_mv = ModeUI_SelectValueFrom("VOLT TO", "mV", kStairMv, COUNT_OF(kStairMv), COUNT_OF(kStairMv) - 1u, from_mv);
        uint8_t to_i = IndexOf(kStairMv, COUNT_OF(kStairMv), to_mv);
        if (to_i < from_i) { // 逆に選んだら入れ替える
            uint8_t t = from_i;
            from_i = to_i;
            to_i = t;
            from_mv = kStairMv[from_i];
            to_mv = kStairMv[to_i];
        }
        s_stair_levels = (uint8_t)(to_i - from_i + 1u);
        s_stair_to_v = to_mv / 1000.0f;
        s_t_acc_ms = (uint32_t)ModeUI_SelectValueFrom("T_ACC", "ms", kSelectTaccMs, COUNT_OF(kSelectTaccMs),
                                                      SYSID_STAIR_TACC_START, 2000.0f);
    } else if (s_test == SYSID_E1T) {
        s_rotate = false;
        static const float kSets[] = { 1.0f, 2.0f, 3.0f, 4.0f, 5.0f };
        s_e1t_set = (uint8_t)(ModeUI_SelectValue("SET(1 B 1.0m/s, 2 A 0.5m/s, 3 C 1.5m/s, 4 D 1.0m/s 270deg, 5 E 0.5m/s 270deg)",
                                                 "", kSets, COUNT_OF(kSets), 1.0f) - 1.0f);
        if (s_e1t_set == 1u) {
            s_e1t_dv = kE1tDvA;
            s_e1t_dv_count = COUNT_OF(kE1tDvA);
        } else if (s_e1t_set == 2u) {
            s_e1t_dv = kE1tDvC;
            s_e1t_dv_count = COUNT_OF(kE1tDvC);
        } else if (s_e1t_set == 3u) {
            s_e1t_dv = kE1tDvD;
            s_e1t_dv_count = COUNT_OF(kE1tDvD);
        } else if (s_e1t_set == 4u) {
            s_e1t_dv = kE1tDvE;
            s_e1t_dv_count = COUNT_OF(kE1tDvE);
        } else {
            s_e1t_set = 0u;
            s_e1t_dv = kE1tDvB;
            s_e1t_dv_count = COUNT_OF(kE1tDvB);
        }
    } else if (s_test == SYSID_E1K0) {
        s_rotate = false;
        static const float kSets[] = { 1.0f, 2.0f };
        s_e1k0_vd = (ModeUI_SelectValue("SET(1 speeds V_d=0, 2 V_d sweep 1.0m/s)", "", kSets, COUNT_OF(kSets), 1.0f) ==
                     2.0f);
    } else if (s_test == SYSID_E1W) {
        s_rotate = false;
        static const float kSets[] = { 1.0f, 2.0f, 3.0f };
        s_e1w_set = (uint8_t)(ModeUI_SelectValue("SET(1 1.0->0.5->1.5m/s, 2 0.3->0.75->2.0m/s, 3 one speed)", "", kSets,
                                                 COUNT_OF(kSets), 1.0f) - 1.0f);
        if (s_e1w_set == 2u) { // 速さを1つ選ぶ(選ぶ表は小さい順)
            static const float kSpeeds[] = { 300.0f, 500.0f, 750.0f, 1000.0f, 1500.0f, 2000.0f };
            float v = ModeUI_SelectValue("SPEED", "mm/s", kSpeeds, COUNT_OF(kSpeeds), kSpeeds[0]);
            s_e1w_speed = IndexOf(kE1wSpeed, COUNT_OF(kE1wSpeed), v);
        }
    } else if (s_test == SYSID_E1B) {
        s_rotate = false; // 選ぶものはない(表の順に進む)
    } else if (s_test == SYSID_E1V) {
        s_rotate = false;
        static const float kSets[] = { 1.0f, 2.0f, 3.0f };
        s_e1v_set = (uint8_t)ModeUI_SelectValue("RUNS(1 all, 2 #1-3-4-6, 3 #2-5)", "", kSets, COUNT_OF(kSets), 1.0f);
    } else if (s_test == SYSID_E1S) {
        static const float kPlaces[] = { 1.0f, 2.0f };
        s_lifted = (ModeUI_SelectValue("PLACE(1 floor, 2 lifted)", "", kPlaces, COUNT_OF(kPlaces), 1.0f) == 2.0f);
        s_rotate = false;
        from_mv = ModeUI_SelectValue("VOLT FROM", "mV", kE1sMv, COUNT_OF(kE1sMv), kE1sMv[0]);
        to_mv = ModeUI_SelectValueFrom("VOLT TO", "mV", kE1sMv, COUNT_OF(kE1sMv), COUNT_OF(kE1sMv) - 1u, from_mv);
        if (to_mv < from_mv) { // 逆に選んだら入れ替える
            float t = from_mv;
            from_mv = to_mv;
            to_mv = t;
        }
        // 浮かせたときの段の数(表は SYSID_E1S_INC_V 刻み)
        s_stair_levels = (uint8_t)(IndexOf(kE1sMv, COUNT_OF(kE1sMv), to_mv) - IndexOf(kE1sMv, COUNT_OF(kE1sMv), from_mv) + 1u);
        s_from_v = from_mv / 1000.0f;
        s_stair_to_v = to_mv / 1000.0f;
        s_t_acc_ms = (uint32_t)ModeUI_SelectValueFrom("T_ACC", "ms", kSelectTaccMs, COUNT_OF(kSelectTaccMs),
                                                      SYSID_E1S_TACC_START, 3000.0f);
    } else {
        s_rotate = (s_test == SYSID_E2);
        from_mv = ModeUI_SelectValue("VOLT FROM", "mV", kSelectMv, COUNT_OF(kSelectMv), kSelectMv[0]);
        uint8_t from_i = IndexOf(kSelectMv, COUNT_OF(kSelectMv), from_mv);
        to_mv = ModeUI_SelectValueFrom("VOLT TO", "mV", kSelectMv, COUNT_OF(kSelectMv), from_i, from_mv);
        if (to_mv < from_mv) { // 逆に選んだら入れ替える
            float t = from_mv;
            from_mv = to_mv;
            to_mv = t;
        }
        s_t_acc_ms = (uint32_t)ModeUI_SelectValueFrom("T_ACC", "ms", kSelectTaccMs, COUNT_OF(kSelectTaccMs),
                                                      SYSID_SELECT_TACC_START, 1000.0f);
    }
    uint8_t repeat = 1u; // E1S・E1V・E1B は条件ごとに1本(もう一度取るときは、クリックで戻るか2周目で)
    if (s_test != SYSID_E1S && s_test != SYSID_E1V && s_test != SYSID_E1B && s_test != SYSID_E1T &&
        s_test != SYSID_E1K0 && s_test != SYSID_E1W) {
        repeat = (uint8_t)ModeUI_SelectValueFrom("REPEAT", "times", kSelectRepeat, COUNT_OF(kSelectRepeat),
                                                 SYSID_SELECT_REPEAT_START, 1.0f);
    }
    uint8_t n = BuildPlan(from_mv, to_mv, repeat);
    if (s_test == SYSID_E1T) {
        printf("%s set %c: v %.0f mm/s (accel %.0f mm/s2, straight %lu ms) -> step V_d = V_d0 +- dV (V_d0 = mean V_d of the"
               " last %lu ms of seg 2, no yaw FB) for %lu ms or %.0f deg -> stop (decel %.0f mm/s2), %u runs:",
               TestName(s_test), "BACDE"[s_e1t_set], kE1tSetSpeed[s_e1t_set], kE1tSetAccel[s_e1t_set],
               (unsigned long)kE1tSetStraightMs[s_e1t_set], (unsigned long)kE1tSetMeasMs[s_e1t_set],
               (unsigned long)kE1tSetTurnMs[s_e1t_set], kE1tSetTurnDeg[s_e1t_set], SYSID_E1T_DECEL_MM_S2, (unsigned)n);
        for (uint8_t k = 0; k < n; k++) printf(" %+.2f", s_e1t_dv[s_plan[k].step_no - 1u]);
        printf(" V\r\n");
        printf("  speed: V_s = V_FF + %.2f e + %.2f int(e) (|I| <= %.2f V), yaw FB (seg 1, 2, 4): V_d = -%.2f theta - %.2f omega\r\n",
               SYSID_E1V_KP, SYSID_E1V_KI, SYSID_E1T_I_MAX, SYSID_E1V_K_THETA, SYSID_E1V_K_OMEGA);
        printf("  abort: dist %.0f mm / |angle| > %.0f deg / |omega| > %.0f dps / |V_L| or |V_R| > vbat / front IR > %.0f for %lu ms\r\n",
               SYSID_E1T_ABORT_DIST_MM, kE1tSetAbortAngle[s_e1t_set], SYSID_E1T_ABORT_OMEGA_DPS, SYSID_E1T_OBSTACLE_TH,
               (unsigned long)SYSID_E1T_OBSTACLE_MS);
    } else if (s_test == SYSID_E1K0) {
        if (s_e1k0_vd) {
            printf("%s V_d sweep: v %.0f mm/s (accel %.0f mm/s2) -> straight %lu ms (theta reset, yaw FB) -> V_d = const"
                   " (no yaw FB, no V_d0) for %lu ms -> stop (theta reset, decel %.0f mm/s2), %u runs:",
                   TestName(s_test), SYSID_E1K0_VD_SPEED_MM_S, SYSID_E1K0_VD_ACCEL_MM_S2,
                   (unsigned long)SYSID_E1K0_CRUISE_MS, (unsigned long)SYSID_E1K0_VD_FREE_MS, SYSID_E1T_DECEL_MM_S2,
                   (unsigned)n);
            for (uint8_t k = 0; k < n; k++) printf(" %+.2f", kE1k0Vd[s_plan[k].step_no - 1u]);
            printf(" V\r\n");
        } else {
            printf("%s: accel -> straight %lu ms (theta reset, yaw FB, V_d0 = mean V_d) -> V_d = 0 exactly (no yaw FB, no V_d0)"
                   " -> stop (theta reset, decel %.0f mm/s2), %u runs:",
                   TestName(s_test), (unsigned long)SYSID_E1K0_CRUISE_MS, SYSID_E1T_DECEL_MM_S2, (unsigned)n);
            for (uint8_t k = 0; k < n; k++) {
                uint8_t j = (uint8_t)(s_plan[k].step_no - 1u);
                printf(" %.1fm/s(%.0fm/s2,%lums)", s_plan[k].volt / 1000.0f, kE1k0Accel[j] / 1000.0f,
                       (unsigned long)kE1k0FreeMs[j]);
            }
            printf("\r\n");
        }
        printf("  speed: V_s = V_FF + %.2f e + %.2f int(e) (|I| <= %.2f V), yaw FB (seg 1, 2, 4): V_d = -%.2f theta - %.2f omega\r\n",
               SYSID_E1V_KP, SYSID_E1V_KI, SYSID_E1T_I_MAX, SYSID_E1V_K_THETA, SYSID_E1V_K_OMEGA);
        printf("  abort: dist %.0f mm / |angle| > %.0f deg / |omega| > %.0f dps / |V_L| or |V_R| > vbat / front IR > %.0f for %lu ms\r\n",
               SYSID_E1T_ABORT_DIST_MM, SYSID_E1K0_ABORT_ANGLE_DEG, SYSID_E1T_ABORT_OMEGA_DPS, SYSID_E1T_OBSTACLE_TH,
               (unsigned long)SYSID_E1T_OBSTACLE_MS);
    } else if (s_test == SYSID_E1W) {
        printf("%s: accel -> straight (yaw FB, V_d0 = mean V_d of its last part) -> no yaw FB: V_d = V_d0 +A, -A, +A, 0"
               " (seg 3-6) -> stop (seg 7, theta reset, decel %.0f mm/s2), %u runs:\r\n",
               TestName(s_test), SYSID_E1T_DECEL_MM_S2, (unsigned)n);
        for (uint8_t s = 0; s < COUNT_OF(kE1wSpeed); s++) { // 速さごとの値(この SET で走る速さだけ)
            bool used = false;
            for (uint8_t k = 0; k < n; k++) used = used || ((s_plan[k].step_no - 1u) / COUNT_OF(kE1wA) == s);
            if (!used) continue;
            printf("  %.2f m/s: accel %.0f mm/s2, straight %lu ms (V_d0: last %lu ms), %lu ms each, A", kE1wSpeed[s] / 1000.0f,
                   kE1wAccel[s], (unsigned long)kE1wCruiseMs[s], (unsigned long)kE1wMeasMs[s],
                   (unsigned long)kE1wStepMs[s]);
            for (uint8_t a = 0; a < kE1wNA[s] && a < COUNT_OF(kE1wA); a++) printf(" %.1f", kE1wA[a]);
            printf(" V x%u\r\n", (unsigned)SYSID_E1W_REPEAT);
        }
        printf("  speed: V_s = V_FF + %.2f e + %.2f int(e) (|I| <= %.2f V), yaw FB (seg 1, 2, 7): V_d = -%.2f theta - %.2f omega\r\n",
               SYSID_E1V_KP, SYSID_E1V_KI, SYSID_E1T_I_MAX, SYSID_E1V_K_THETA, SYSID_E1V_K_OMEGA);
        printf("  abort: dist %.0f mm / |angle| > %.0f deg / |omega| > %.0f dps / |V_L| or |V_R| > vbat / front IR > %.0f for %lu ms\r\n",
               SYSID_E1W_ABORT_DIST_MM, SYSID_E1W_ABORT_ANGLE_DEG, SYSID_E1W_ABORT_OMEGA_DPS, SYSID_E1T_OBSTACLE_TH,
               (unsigned long)SYSID_E1T_OBSTACLE_MS);
    } else if (s_test == SYSID_E1B) {
        printf("%s: accel %.0f mm/s2 (V_s = %.3f v + %.3f + %.3f a) -> cruise %lu ms -> brake V until v < %.0f mm/s -> 0V, %u runs:",
               TestName(s_test), SYSID_E1B_ACCEL_MM_S2, SYSID_E1V_KV, SYSID_E1V_V0, SYSID_E1V_KA,
               (unsigned long)SYSID_E1B_CRUISE_MS, SYSID_E1B_END_MM_S, (unsigned)n);
        for (uint8_t k = 0; k < n; k++) {
            printf(" #%u(%.1f m/s, %+.1f V)", (unsigned)s_plan[k].step_no, s_plan[k].volt / 1000.0f,
                   kE1bBrakeV[s_plan[k].step_no - 1u]);
        }
        printf("\r\n");
        printf("  yaw FB: V_d = -%.2f theta - %.2f omega (L -V_d, R +V_d), |V_d| <= %.2f V\r\n", SYSID_E1V_K_THETA,
               SYSID_E1V_K_OMEGA, SYSID_E1V_VD_MAX);
        printf("  abort: dist %.0f mm / angle %.0f deg / v < -%.0f mm/s\r\n", SYSID_E1V_ABORT_DIST_MM,
               SYSID_E1V_ABORT_ANGLE_DEG, SYSID_E1B_ABORT_BACK_MM_S);
    } else if (s_test == SYSID_E1V) {
        printf("%s: V_s = %.3f v + %.3f + %.3f a, dV(L+, R-) = %.3f v + %.4f a, cruise %lu ms, %u runs:",
               TestName(s_test), SYSID_E1V_KV, SYSID_E1V_V0, SYSID_E1V_KA, SYSID_E1V_LR_C, SYSID_E1V_LR_A,
               (unsigned long)SYSID_E1V_CRUISE_MS, (unsigned)n);
        for (uint8_t k = 0; k < n; k++) printf(" #%u", (unsigned)s_plan[k].step_no);
        printf("\r\n");
        printf("  yaw FB: V_d = -%.2f theta - %.2f omega (L -V_d, R +V_d), |V_d| <= %.2f V\r\n", SYSID_E1V_K_THETA,
               SYSID_E1V_K_OMEGA, SYSID_E1V_VD_MAX);
        printf("  speed PI: V_s = V_FF + %.2f e + %.2f int(e), |I| <= %.2f V (e in m/s)\r\n", SYSID_E1V_KP,
               SYSID_E1V_KI, SYSID_E1V_I_MAX);
        printf("  abort: dist %.0f mm / v > v_max + %.0f mm/s / angle %.0f deg\r\n", SYSID_E1V_ABORT_DIST_MM,
               SYSID_E1V_ABORT_V_MARGIN_MM_S, SYSID_E1V_ABORT_ANGLE_DEG);
    } else if (s_test == SYSID_E1S && s_lifted) {
        printf("%s LIFTED: forward %.0f-%.0f mV (%u steps in 1 run, 1 hand), T_acc %lu ms, rest %lu ms, abort wheel %.0f mm/s\r\n",
               TestName(s_test), from_mv, to_mv, (unsigned)s_stair_levels, (unsigned long)s_t_acc_ms,
               (unsigned long)SYSID_REST_MS, SYSID_ABORT_WHEEL_MM_S);
    } else if (s_test == SYSID_E1S) {
        printf("%s: forward %.0f-%.0f mV (%u steps, 1 per hand), T_acc %lu ms, abort dist %.0f mm / v %.0f mm/s / angle %.0f deg\r\n",
               TestName(s_test), from_mv, to_mv, (unsigned)n, (unsigned long)s_t_acc_ms, SYSID_E1S_ABORT_DIST_MM,
               SYSID_E1S_ABORT_V_MM_S, SYSID_E1S_ABORT_ANGLE_DEG);
    } else if (s_test == SYSID_E2S) {
        printf("%s %s: %.0f-%.0f mV (%u levels x ccw/cw = %u steps per run), T_acc %lu ms, rest %lu ms, %u runs\r\n",
               TestName(s_test), s_lifted ? "LIFTED" : "floor", from_mv, to_mv, (unsigned)s_stair_levels, (unsigned)(s_stair_levels * 2u),
               (unsigned long)s_t_acc_ms, (unsigned long)SYSID_REST_MS, (unsigned)n);
    } else if (s_test == SYSID_E0) {
        printf("%s %s: x%u pairs = %u runs (ramp %.2f V/s up to %.2f V)\r\n", TestName(s_test),
               s_rotate ? "pivot" : "translation", (unsigned)repeat, (unsigned)n, SYSID_E0_RAMP_V_PER_S,
               SYSID_E0_V_MAX);
    } else {
        printf("%s: %.0f-%.0f mV, T_acc %lu ms, x%u pairs = %u runs\r\n", TestName(s_test), from_mv, to_mv,
               (unsigned long)s_t_acc_ms, (unsigned)repeat, (unsigned)n);
    }
    return n;
}

static void SetupLogger(const char *file) {
    const SysidDebug *d = SysidLoop_GetDebug();

    Logger_Init(); // 先頭に time_s
    Logger_SetDirName("sysid");
    Logger_SetFileName(file);
    Logger_AddField("step", &d->step);
    Logger_AddField("seg", &d->seg);
    bool ff = (s_test == SYSID_E1V || s_test == SYSID_E1B); // logic/sysid/ff_test で走る
    if (ff) {
        Logger_AddField("v_ref", &d->v_ref);
        Logger_AddField("a_ref", &d->a_ref);
    }
    Logger_AddField("v_cmd", &d->v_cmd);
    Logger_AddField("pwm_l", &d->pwm_l);
    Logger_AddField("pwm_r", &d->pwm_r);
    Logger_AddField("vbat", &d->vbat);
    Logger_AddField("vl", &d->vl);
    Logger_AddField("vr", &d->vr);
    Logger_AddField("gyro_z", &d->gyro_z);
    Logger_AddField("gyro_x", &d->gyro_x);
    Logger_AddField("gyro_y", &d->gyro_y);
    Logger_AddField("acc_x", &d->acc_x);
    Logger_AddField("acc_y", &d->acc_y);
    Logger_AddField("acc_z", &d->acc_z);
    Logger_AddField("dist", &d->dist);
    Logger_AddField("angle", &d->angle);
    if (s_test == SYSID_E1T || s_test == SYSID_E1K0 || s_test == SYSID_E1W) { // 仕様の列に v_ref を足した 25 列(100B/行)
        Logger_AddField("v_ref", &d->v_ref);
        Logger_AddField("v_ff", &d->v_ff);
        Logger_AddField("v_p", &d->v_p);
        Logger_AddField("v_i", &d->v_i);
        Logger_AddField("v_d", &d->v_d);
        Logger_AddField("v_d0", &d->v_d0);
        Logger_AddField("dv", &d->dv);
        Logger_AddField("v_d_meas", &d->v_d_meas);
    }
    if (ff) { // 旋回のフィードバック
        Logger_AddField("v_s", &d->v_s);
        Logger_AddField("v_d", &d->v_d);
        Logger_AddField("theta", &d->theta);
    }
    if (s_test == SYSID_E1V) { // 並進の PI(25 列、100B/行)
        Logger_AddField("v_ff", &d->v_ff);
        Logger_AddField("v_p", &d->v_p);
        Logger_AddField("v_i", &d->v_i);
    }
    Logger_SetDecimation(1u); // 1kHz
}

// 進み具合を LED1〜7 に2進数で出す(n 本目。LED1 が一番下の桁。U6 の不良で確実に光るのは LED1〜7)
static void ShowRunNumber(uint8_t n) {
    LED_SetShiftPattern((uint16_t)(n & 0x7Fu));
}

static void PrintResult(const SysidResult *r, bool saved, uint32_t dropped) {
    printf("  %s%s: %lu ms, peak v %.0f mm/s, peak w %.0f dps, dist %.1f mm, angle %.1f deg\r\n",
           r->failsafe ? "FAILSAFE " : "", AbortName(r->abort), (unsigned long)r->ticks, r->peak_v_mm_s,
           r->peak_omega_dps, r->dist_mm, r->angle_deg);
    printf("  vbat at start %.2f V, PWM saturated %lu ticks, log %s (dropped %lu rows)\r\n", r->vbat_start,
           (unsigned long)r->sat_ticks, saved ? "saved" : "SAVE FAILED", (unsigned long)dropped);
    if (r->sat_ticks > 0u) {
        // E1V はファイルが走り始めの tick から始まるので time_s と同じ(E1S は走る前から記録しているのでずれる)
        printf("  PWM first saturated %.3f s after the run start\r\n", (float)r->sat_first_tick * CONTROL_DT_S);
    }
    if (s_test == SYSID_E1T) printf("  v_d_meas (mean V_d, last %lu ms of seg 2) %+.3f V\r\n",
                                    (unsigned long)kE1tSetMeasMs[s_e1t_set], r->v_d_meas);
    if (s_test == SYSID_E1K0) printf("  v_d_meas (mean V_d of seg 2) %+.3f V\r\n", r->v_d_meas);
    if (s_test == SYSID_E1W) printf("  v_d_meas (mean V_d, last part of seg 2) %+.3f V\r\n", r->v_d_meas);
    if (s_test == SYSID_E1V || s_test == SYSID_E1B || s_test == SYSID_E1T || s_test == SYSID_E1K0 ||
        s_test == SYSID_E1W) {
        printf("  V_d limited (|V_d| > %.2f V) %lu ticks", SYSID_E1V_VD_MAX, (unsigned long)r->vd_sat_ticks);
        if (r->vd_sat_ticks > 0u) printf(", first at %.3f s (time_s)", (float)r->vd_sat_first_tick * CONTROL_DT_S);
        printf("\r\n");
    }
    if (s_test == SYSID_E0) {
        printf("  start voltage: L %s%.3f V, R %s%.3f V\r\n", r->moved_l ? "" : "(not moved) ", r->move_v_l,
               r->moved_r ? "" : "(not moved) ", r->move_v_r);
    }
    if (s_test == SYSID_E2S || (s_test == SYSID_E1S && s_lifted)) {
        printf("  steps: ended at %u / %u%s\r\n", (unsigned)r->step, (unsigned)r->steps,
               (r->abort != STEP_ABORT_NONE || r->failsafe) ? " (the rest were skipped)" : "");
    }
}

// E1S: 1周を1つのファイルにする。開いたままなら true(次の1本は続きに記録する)
static bool s_file_open = false;

// ログのファイルを開く(log_resume なら開いたままのファイルの続きに書くので、何もしない)。開けなければ知らせて false
static bool OpenLog(bool log_resume) {
    if (log_resume) {
        printf("  log: continued in the same file\r\n");
        return true;
    }
    SetupLogger(s_file_name);
    char path[SDCARD_PATH_MAX];
    if (!Logger_StreamBegin(path, sizeof(path))) {
        printf("  SD: cannot open a log file (not run)\r\n");
        ModeUI_WaitClickBlinking(MODE_UI_LED_SD_ERROR);
        return false;
    }
    printf("  log: %s\r\n", path);
    s_file_open = (s_test == SYSID_E1S);
    return true;
}

// ms 待つ。記録を流している間に SD へブロックを送る
static void WaitPolling(uint32_t ms) {
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < ms) {
        Logger_StreamPoll();
        HAL_Delay(1);
    }
}

// 1本走って、ログを閉じる(E1S は周の最後の1本 last か、中止したときだけ閉じる)。フェイルセーフなら閉じてから戻らない。
// E1S で中止して、周の残りを飛ばしたら true
static bool RunOne(const RunPlan *p, bool last) {
    bool one_file = (s_test == SYSID_E1S);
    const char *dir = DirName(s_rotate, p->sign);
    if (s_test == SYSID_E2S) {
        snprintf(s_file_name, sizeof(s_file_name), "%s_%d_%dmv", s_lifted ? "e2sl" : "e2s",
                 (int)lroundf(p->volt * 1000.0f),
                 (int)lroundf(s_stair_to_v * 1000.0f));
    } else if (s_test == SYSID_E1S) {
        snprintf(s_file_name, sizeof(s_file_name), "%s_%d_%dmv", s_lifted ? "e1sl" : "e1s", (int)lroundf(s_from_v * 1000.0f),
                 (int)lroundf(s_stair_to_v * 1000.0f));
    } else if (s_test == SYSID_E1T) { // 例: e1t3_1000mms_p030(速さ[mm/s]、ΔV の向き p = +・m = −、|ΔV| の 0.01V)
        // 60° より大きく回す組(D・E)は後ろに回す角度を付ける(例 e1t3_1000mms_p065_a270)
        float dv = s_e1t_dv[p->step_no - 1u];
        int n = snprintf(s_file_name, sizeof(s_file_name), "e1t3_%dmms_%c%03d", (int)lroundf(p->volt),
                         (dv < 0.0f) ? 'm' : 'p', (int)lroundf(fabsf(dv) * 100.0f));
        if (kE1tSetTurnDeg[s_e1t_set] > 60.0f && n > 0 && (size_t)n < sizeof(s_file_name)) {
            snprintf(&s_file_name[n], sizeof(s_file_name) - (size_t)n, "_a%d", (int)lroundf(kE1tSetTurnDeg[s_e1t_set]));
        }
    } else if (s_test == SYSID_E1W) { // 例: e1w_1000mms_a020(速さ[mm/s]、A の 0.01V)
        float amp = kE1wA[(p->step_no - 1u) % COUNT_OF(kE1wA)];
        snprintf(s_file_name, sizeof(s_file_name), "e1w_%dmms_a%03d", (int)lroundf(p->volt), (int)lroundf(amp * 100.0f));
    } else if (s_test == SYSID_E1K0 && s_e1k0_vd) { // 例: e1k0_1000mms_m005(V_d の向き p = +・m = −、|V_d| の 0.01V)
        float vd = kE1k0Vd[p->step_no - 1u];
        snprintf(s_file_name, sizeof(s_file_name), "e1k0_%dmms_%c%03d", (int)lroundf(p->volt), (vd < 0.0f) ? 'm' : 'p',
                 (int)lroundf(fabsf(vd) * 100.0f));
    } else if (s_test == SYSID_E1K0) { // 例: e1k0_1000mms(速さ[mm/s])
        snprintf(s_file_name, sizeof(s_file_name), "e1k0_%dmms", (int)lroundf(p->volt));
    } else if (s_test == SYSID_E1V) { // 例: e1vp_2000_a5_c0(並進の PI と旋回の FB あり。v_max[mm/s]、加速度[m/s²]、左右の補正 LR_C[mV/(m/s)])
        snprintf(s_file_name, sizeof(s_file_name), "e1vp_%d_a%d_c%d", (int)lroundf(p->volt),
                 (int)lroundf(kE1vAccel[p->step_no - 1u] / 1000.0f), (int)lroundf(SYSID_E1V_LR_C * 1000.0f));
    } else if (s_test == SYSID_E1B) { // 例: e1b_1500_m800(v0[mm/s]、ブレーキの電圧 −0.8V。0V なら m0)
        snprintf(s_file_name, sizeof(s_file_name), "e1b_%d_m%d", (int)lroundf(p->volt),
                 (int)lroundf(-kE1bBrakeV[p->step_no - 1u] * 1000.0f));
    } else if (s_test == SYSID_E0) {
        snprintf(s_file_name, sizeof(s_file_name), "e0%c_%s", s_rotate ? 'r' : 't', dir);
    } else {
        snprintf(s_file_name, sizeof(s_file_name), "e%d_%s_%dmv", (int)s_test, dir, (int)lroundf(p->volt * 1000.0f));
    }

    bool e1s = (s_test == SYSID_E1S);
    bool log_resume = one_file && s_file_open;
    // E1S は走る前の1秒(手を離す時間と6軸の測り直し)も記録する(seg −1、step 0)。止まっているときの加速度の平均を
    // 後で引けるように。その区間のジャイロは、測り直しが終わるまで前のゼロ点を引いている
    if (e1s) {
        if (!OpenLog(log_resume)) return false;
        if (log_resume) {
            Logger_Resume();
        } else {
            Logger_Start();
        }
    }

    WaitPolling(SYSID_START_DELAY_MS - GYRO_RECAL_MS); // 手を離す時間
    {
        // 6軸の止まっているときの値を測り直す。加速度は機体の座標で出す(ログの acc_* から引けば、止まっていたときからの変化)
        ImuCalib cal = SysidLoop_RecalibrateImu(GYRO_RECAL_MS);
        float rest[3];
        Imu_RestAccBody(&cal, rest);
        printf("  gyro bias [LSB]: x %.1f y %.1f z %.1f\r\n", cal.gyro_bias[0], cal.gyro_bias[1], cal.gyro_bias[2]);
        printf("  acc at rest [m/s2]: x %+.3f y %+.3f z %+.3f (|a| %.3f)\r\n", rest[0], rest[1], rest[2],
               sqrtf(rest[0] * rest[0] + rest[1] * rest[1] + rest[2] * rest[2]));
    }

    if (!e1s && !OpenLog(false)) return false;

    bool e1s_floor = e1s && !s_lifted;
    bool e1s_lifted = e1s && s_lifted;
    StepConfig cfg = {
        .type = (s_test == SYSID_E0) ? STEP_TEST_RAMP : STEP_TEST_STEP,
        .rotate = s_rotate,
        .sign = p->sign,
        .volt = p->volt,
        .t_acc_ms = s_t_acc_ms,
        .levels = (s_test == SYSID_E2S || e1s_lifted) ? s_stair_levels : 1u,
        .v_inc = (s_test == SYSID_E2S) ? SYSID_STAIR_INC_V : (e1s_lifted ? SYSID_E1S_INC_V : 0.0f),
        .alternate = (s_test == SYSID_E2S),
        .no_reverse = (s_test == SYSID_E2S) || e1s, // 階段・E1S は −V で止めず、0V で止まるのを待つ
        // 浮かせた E1S は進まないので距離・速度・角度を見ない(車輪の速さの上限と、止まらないときの中止は残る)
        .no_dist_abort = e1s_lifted,
        .abort_dist_mm = e1s_floor ? SYSID_E1S_ABORT_DIST_MM : 0.0f,
        .abort_v_mm_s = e1s_floor ? SYSID_E1S_ABORT_V_MM_S : 0.0f,
        .abort_angle_deg = e1s_floor ? SYSID_E1S_ABORT_ANGLE_DEG : 0.0f,
        .step_base = e1s ? (uint16_t)(p->step_no - 1u) : 0u,
        .skip_pre = e1s && log_resume, // 区間0 はファイルの始まりの目印なので、1本目だけ
        .rest_after = e1s,             // 止まった後の休み(区間4)まで記録する
    };
    if (s_test == SYSID_E1V || s_test == SYSID_E1B || s_test == SYSID_E1T || s_test == SYSID_E1K0 ||
        s_test == SYSID_E1W) {
        bool e1b = (s_test == SYSID_E1B);
        bool e1k0 = (s_test == SYSID_E1K0);
        bool e1w = (s_test == SYSID_E1W);
        // E1K0・E1W も E1T と同じ走り方(E1K0 の区間3 は V_d = 0。E1W は下で段の並びに書き換える)
        bool e1t = (s_test == SYSID_E1T) || e1k0 || e1w;
        float accel = e1b ? SYSID_E1B_ACCEL_MM_S2 : (e1t ? kE1tSetAccel[s_e1t_set] : kE1vAccel[p->step_no - 1u]);
        uint32_t cruise_ms = e1b ? SYSID_E1B_CRUISE_MS : (e1t ? kE1tSetStraightMs[s_e1t_set] : SYSID_E1V_CRUISE_MS);
        // E1K0: 区間3 の V_d(速さの組は 0、V_d を振る組は表の値)と長さ
        float k0_vd = 0.0f;
        uint32_t k0_free_ms = 0u;
        if (e1k0) {
            accel = s_e1k0_vd ? SYSID_E1K0_VD_ACCEL_MM_S2 : kE1k0Accel[p->step_no - 1u];
            cruise_ms = SYSID_E1K0_CRUISE_MS;
            k0_vd = s_e1k0_vd ? kE1k0Vd[p->step_no - 1u] : 0.0f;
            k0_free_ms = s_e1k0_vd ? SYSID_E1K0_VD_FREE_MS : kE1k0FreeMs[p->step_no - 1u];
        }
        FfConfig ff = {
            .v_max_mm_s = p->volt,
            .accel_mm_s2 = accel,
            .decel_mm_s2 = e1t ? SYSID_E1T_DECEL_MM_S2 : 0.0f, // 0 なら加速と同じ
            .cruise_ms = cruise_ms,
            .kv = SYSID_E1V_KV / 1000.0f, // [V/(m/s)] → [V/(mm/s)]
            .v0 = SYSID_E1V_V0,
            .ka = SYSID_E1V_KA / 1000.0f, // [V/(m/s²)] → [V/(mm/s²)]
            .lr_c = SYSID_E1V_LR_C / 1000.0f, // [V/(m/s)] → [V/(mm/s)]
            .lr_a = SYSID_E1V_LR_A / 1000.0f, // [V/(m/s²)] → [V/(mm/s²)]
            .k_theta = SYSID_E1V_K_THETA,
            .k_omega = SYSID_E1V_K_OMEGA,
            .vd_max = SYSID_E1V_VD_MAX,
            .kp = e1b ? 0.0f : SYSID_E1V_KP / 1000.0f, // [V/(m/s)] → [V/(mm/s)]。E1B の並進は FF だけ
            .ki = e1b ? 0.0f : SYSID_E1V_KI / 1000.0f, // [V/m] → [V/mm]
            .i_max = e1t ? SYSID_E1T_I_MAX : SYSID_E1V_I_MAX,
            .brake_test = e1b,
            .brake_v = e1b ? kE1bBrakeV[p->step_no - 1u] : 0.0f,
            .brake_end_mm_s = SYSID_E1B_END_MM_S,
            .turn_test = e1t,
            .turn_dv = e1k0 ? k0_vd : (e1t ? s_e1t_dv[p->step_no - 1u] : 0.0f), // E1K0 は V_d = turn_dv ちょうど
            .turn_ms = e1k0 ? k0_free_ms : kE1tSetTurnMs[s_e1t_set],
            .turn_deg = e1k0 ? 0.0f : kE1tSetTurnDeg[s_e1t_set], // E1K0 は時間だけで終わる(角度は中止で見る)
            .turn_no_vd0 = e1k0,
            .cruise_theta_reset = e1k0,
            .vd_meas_ms = e1k0 ? SYSID_E1K0_CRUISE_MS : kE1tSetMeasMs[s_e1t_set], // E1K0 は一定の区間の全部
            // E1T は速さを見ず、距離・角度(E1T の上限)・角速度・電圧・前の障害物を見る
            .abort_dist_mm = e1t ? SYSID_E1T_ABORT_DIST_MM : SYSID_E1V_ABORT_DIST_MM,
            .abort_v_mm_s = (e1b || e1t) ? 0.0f : p->volt + SYSID_E1V_ABORT_V_MARGIN_MM_S, // E1B は速さを見ない
            .abort_back_mm_s = e1b ? SYSID_E1B_ABORT_BACK_MM_S : 0.0f,
            .abort_angle_deg = e1k0 ? SYSID_E1K0_ABORT_ANGLE_DEG
                                    : (e1t ? kE1tSetAbortAngle[s_e1t_set] : SYSID_E1V_ABORT_ANGLE_DEG),
            .abort_omega_dps = e1t ? SYSID_E1T_ABORT_OMEGA_DPS : 0.0f,
            .abort_volt_over = e1t,
            .abort_front_ir = e1t ? SYSID_E1T_OBSTACLE_TH : 0.0f,
            .abort_front_ms = SYSID_E1T_OBSTACLE_MS,
            .step_no = p->step_no,
        };
        if (e1w) { // V_d0 + A → V_d0 − A → V_d0 + A → V_d0 を STEP_MS ずつ(角度では終わらない)
            uint8_t j = (uint8_t)(p->step_no - 1u);
            uint8_t s = (uint8_t)(j / COUNT_OF(kE1wA)); // 速さの番号
            float amp = kE1wA[j % COUNT_OF(kE1wA)];
            ff.accel_mm_s2 = kE1wAccel[s];
            ff.cruise_ms = kE1wCruiseMs[s];
            ff.vd_meas_ms = kE1wMeasMs[s];
            ff.turn_dv = 0.0f;
            ff.turn_ms = kE1wStepMs[s];
            ff.turn_deg = 0.0f;
            ff.turn_seq_n = 4u;
            ff.turn_seq_dv[0] = amp;
            ff.turn_seq_dv[1] = -amp;
            ff.turn_seq_dv[2] = amp;
            ff.turn_seq_dv[3] = 0.0f;
            ff.abort_dist_mm = SYSID_E1W_ABORT_DIST_MM;
            ff.abort_angle_deg = SYSID_E1W_ABORT_ANGLE_DEG;
            ff.abort_omega_dps = SYSID_E1W_ABORT_OMEGA_DPS;
        }
        SysidLoop_StartFfRun(&ff); // 記録も同じ tick から始まる
    } else {
        // 記録も同じ tick から始まる(E1S はもう記録している)
        SysidLoop_StartRun(&cfg, e1s ? SYSID_LOG_KEEP : SYSID_LOG_START);
    }
    // 終わりは割り込みが決める(区間3・4・9 の後、またはフェイルセーフ)。どの区間にも終わりがあるので打ち切りの時間は置かない
    while (!SysidLoop_IsDone()) {
        Logger_StreamPoll();
        HAL_Delay(1);
    }
    SysidResult r = SysidLoop_GetResult();

    bool round_aborted = one_file && (r.abort != STEP_ABORT_NONE); // E1S: 中止したら周の残りを飛ばす
    if (s_file_open && !last && !r.failsafe && !round_aborted) {
        // E1S の周の途中: ファイルは閉じず、いっぱいのブロックだけ送っておく(置き直している間は送らないため)
        bool ok = Logger_StreamDrain(3000u);
        LoggerStreamStats st;
        Logger_StreamGetStats(&st);
        PrintResult(&r, ok, st.dropped_rows);
        if (ok) {
            LED_SetDirectPattern(LED_DIRECT_ALL);
            HAL_Delay(500);
            LED_SetDirectPattern(0x00u);
            return false;
        }
        // 書けなかったら閉じる(次の1本は新しいファイルになる)
        Logger_StreamEnd(NULL);
        s_file_open = false;
        ModeUI_WaitClickBlinking(MODE_UI_LED_SD_ERROR);
        return false;
    }

    uint32_t dropped = 0;
    bool saved = Logger_StreamEnd(&dropped);
    s_file_open = false;
    PrintResult(&r, saved, dropped);
    if (round_aborted && !last) {
        printf("  file closed (aborted: the rest of the round was skipped)\r\n");
    } else if (one_file) {
        printf("  file closed (end of the round)\r\n");
    }
    if (r.failsafe) FailSafe_Halt(); // ファイルは閉じた

    if (saved) {
        LED_SetDirectPattern(LED_DIRECT_ALL); // 保存できた合図(ModeUI_SaveLogToSD と同じ)
        HAL_Delay(500);
        LED_SetDirectPattern(0x00u);
    } else {
        ModeUI_WaitClickBlinking(MODE_UI_LED_SD_ERROR);
    }
    return round_aborted;
}

void SysidMode_Run(SysidTest test) {
    if (FailSafe_IsTripped()) FailSafe_Halt(); // 起動時の低電圧など
    s_test = test;
    if (!SDCard_IsMounted()) printf("WARNING: SD card is not mounted (logs cannot be saved)\r\n");

    uint8_t n = SelectSettings();
    uint8_t i = 0;
    while (1) {
        const RunPlan *p = &s_plan[i];
        if (s_test == SYSID_E1V) {
            printf("next %u/%u: %s #%u accel %.0f mm/s2, v_max %.0f mm/s (hand: RUN, click: back one)\r\n",
                   (unsigned)(i + 1u), (unsigned)n, TestName(s_test), (unsigned)p->step_no,
                   kE1vAccel[p->step_no - 1u], p->volt);
        } else if (s_test == SYSID_E1B) {
            printf("next %u/%u: %s #%u v0 %.0f mm/s, brake %+.2f V (hand: RUN, click: back one)\r\n",
                   (unsigned)(i + 1u), (unsigned)n, TestName(s_test), (unsigned)p->step_no, p->volt,
                   kE1bBrakeV[p->step_no - 1u]);
        } else if (s_test == SYSID_E1T) {
            float dv = s_e1t_dv[p->step_no - 1u];
            // + は左に曲がるので右の壁ぎわから、− は左の壁ぎわから出る
            printf("next %u/%u: %s v %.0f mm/s, dV %+.2f V, start at the %s wall (hand: RUN, click: back one)\r\n",
                   (unsigned)(i + 1u), (unsigned)n, TestName(s_test), p->volt, dv, (dv > 0.0f) ? "RIGHT" : "LEFT");
        } else if (s_test == SYSID_E1W) {
            // 左右に振れるので真ん中から出る
            printf("next %u/%u: %s v %.0f mm/s, A %.2f V, start at the CENTER (hand: RUN, click: back one)\r\n",
                   (unsigned)(i + 1u), (unsigned)n, TestName(s_test), p->volt,
                   kE1wA[(p->step_no - 1u) % COUNT_OF(kE1wA)]);
        } else if (s_test == SYSID_E1K0 && s_e1k0_vd) {
            // 右に曲がる本もあるので真ん中から出る
            printf("next %u/%u: %s v %.0f mm/s, V_d %+.2f V for %lu ms, start at the CENTER (hand: RUN, click: back one)\r\n",
                   (unsigned)(i + 1u), (unsigned)n, TestName(s_test), p->volt, kE1k0Vd[p->step_no - 1u],
                   (unsigned long)SYSID_E1K0_VD_FREE_MS);
        } else if (s_test == SYSID_E1K0) {
            // 左に曲がる見込みなので右の壁ぎわから出る
            printf("next %u/%u: %s v %.0f mm/s, V_d = 0 for %lu ms, start at the RIGHT wall (hand: RUN, click: back one)\r\n",
                   (unsigned)(i + 1u), (unsigned)n, TestName(s_test), p->volt,
                   (unsigned long)kE1k0FreeMs[p->step_no - 1u]);
        } else if (s_test == SYSID_E1S && s_lifted) {
            printf("next: %s LIFTED fwd %.1f-%.1f V, %u steps (hand: RUN)\r\n", TestName(s_test), p->volt,
                   s_stair_to_v, (unsigned)s_stair_levels);
        } else if (s_test == SYSID_E1S) {
            printf("next %u/%u: %s step %u fwd %.2f V (hand: RUN, click: back one)\r\n", (unsigned)(i + 1u),
                   (unsigned)n, TestName(s_test), (unsigned)p->step_no, p->volt);
        } else if (s_test == SYSID_E2S) {
            printf("next %u/%u: %s %.1f-%.1f V, ccw/cw each (hand: RUN, click: back one)\r\n", (unsigned)(i + 1u),
                   (unsigned)n, TestName(s_test), p->volt, s_stair_to_v);
        } else if (s_test == SYSID_E0) {
            printf("next %u/%u: %s %s %s (hand: RUN, click: back one)\r\n", (unsigned)(i + 1u), (unsigned)n,
                   TestName(s_test), s_rotate ? "pivot" : "translation", DirName(s_rotate, p->sign));
        } else {
            printf("next %u/%u: %s %s %.2f V (hand: RUN, click: back one)\r\n", (unsigned)(i + 1u), (unsigned)n,
                   TestName(s_test), DirName(s_rotate, p->sign), p->volt);
        }
        ShowRunNumber((uint8_t)(i + 1u));

        if (ModeUI_WaitHandStartOrClick()) { // クリック: 1本前に戻る
            if (i > 0u) i--;
            continue;
        }
        LED_SetShiftPattern(0x0000u);
        if (RunOne(p, (uint8_t)(i + 1u) >= n)) { // E1S で中止: 周の残りを飛ばして1本目から
            printf("round aborted at %u/%u. starting again from 1\r\n", (unsigned)(i + 1u), (unsigned)n);
            i = 0;
            continue;
        }

        i++;
        if (i >= n) {
            printf("all %u runs done. starting again from 1\r\n", (unsigned)n);
            i = 0;
        }
    }
}
