#ifndef INC_FF_TEST_H_
#define INC_FF_TEST_H_

#include "global.h"
#include "params.h"
#include "logic/sysid/step_test.h" // StepMeas・StepAbort を使う

// 同定したモデルの FF だけで走る検証(E1V)の状態機械。ハードには触らない(測った値を受け取り、かける電圧を返すだけ)。
// 台形の速度パターン(v_max まで加速度 accel → v_max で cruise_ms → 減速度 accel で 0)から v_ref・a_ref を出し、
//   V = kv · v_ref + v0 + ka · a_ref (v_ref > 0 の間。v_ref が 0 になったら 0V = ショートブレーキ)
// を左右の補正 ΔV = lr_c · v_ref + lr_a · a_ref と、旋回のフィードバック V_d = −k_theta · θ − k_omega · ω(|V_d| ≤ vd_max)で
//   V_L = V + ΔV − V_d、V_R = V − ΔV + V_d
// に分ける(V_d > 0 で左に曲がる向き。θ・ω は走り始めからのジャイロの積分角・角速度[rad]・[rad/s]、+ が左)。
// 並進は V = v_ff + kp·e + ki·∫e dt(e = v_ref − v、v = (vl + vr)/2、|ki·∫e dt| ≤ i_max。v_ref > 0 の間だけ。kp・ki が 0 なら FF だけ)。
// ゲインが 0 なら旋回もフィードバックなし。ブレーキの試験の区間3 は一定の電圧で、並進の PI はかけない。
// 減速中も車輪は前転なので、v0 の向きは変えない(V は負になる)。ΔV は v_ref に比例するので、減速でも向きは変わらない。
//
// 区間(ログの seg 列):
//   0 始まり  0V を SYSID_PRE_MS
//   1 加速    v_ref = accel · t(v_max まで)
//   2 一定    v_ref = v_max を cruise_ms
//   3 減速    v_ref = v_max − accel · t(0 まで)
//             ブレーキの試験(brake_test。E1B)では、一定の電圧 V = brake_v を、並進の速度が brake_end_mm_s 未満になるまで
//             (v_ref・a_ref は 0。旋回のフィードバックは続ける。SYSID_STOP_TIMEOUT_MS を過ぎたら中止)
//   4 停止    0V で止まるまで(SYSID_STOP_* の並進の判定。SYSID_STOP_TIMEOUT_MS で打ち切り)
//   9 中止    0V で止まるまで(区間1〜3で距離・速さ・角度が上限を超えた、または後ろに戻った)
//
// 旋回のステップの試験(turn_test。E1T)では、一定の区間と減速の間に「ステップ」(FF_SEG_TURN)を挟む:
//   加速 → 一定(最後の vd_meas_ms の V_d の平均を v_d_meas にする) → ステップ(旋回の FB をやめ、V_d = V_d0 + turn_dv、
//   V_d0 = v_d_meas。turn_ms たつか、ステップの間の回転が turn_deg を越えたら終わる) →
//   減速(FB の θ をここで 0 に入れ直す) → 停止。並進の FF + PI は止まるまで続ける。
//   ログの seg 列は FfTest_LogSeg で 1 加速、2 一定、3 ステップ、4 減速、5 停止、9 中止 にする。
//   E1K0(κ₀ を測る)では turn_no_vd0 で V_d = turn_dv(0)ちょうどにし、cruise_theta_reset で一定の区間の始めに θ を 0 に入れ直す。
//   E1W(b_ω を測る)では turn_seq_n(2 以上)で、ステップを turn_seq_dv[] の順に turn_ms ずつ続ける(V_d = V_d0 + turn_seq_dv[i]。
//   角度では終わらない)。ログの seg 列は 1 加速、2 一定、3〜(2 + n) ステップの i 番目、3 + n 減速、4 + n 停止、9 中止
//   (n = 4 なら 3・4・5・6 がステップ、7 減速、8 停止)。seg は V_d を変えた tick と同じ tick で変わる。

#define FF_TURN_SEQ_MAX 4u

typedef enum {
    FF_SEG_PRE = 0,
    FF_SEG_ACC = 1,
    FF_SEG_CRUISE = 2,
    FF_SEG_DEC = 3,
    FF_SEG_STOP = 4,
    FF_SEG_TURN = 5,  // 旋回のステップ(turn_test だけ)
    FF_SEG_ABORT = 9,
} FfSegment;

typedef struct {
    float v_max_mm_s;
    float accel_mm_s2;
    float decel_mm_s2;     // 区間3(減速。E1T は区間4)の減速度(0 なら accel_mm_s2 と同じ)
    uint32_t cruise_ms;
    float kv;      // [V/(mm/s)]
    float v0;      // [V]
    float ka;      // [V/(mm/s²)]
    float lr_c;    // 左右の補正の v_ref の係数[V/(mm/s)]
    float lr_a;    // 左右の補正の a_ref の係数[V/(mm/s²)]
    float k_theta; // 旋回のフィードバックの角度のゲイン[V/rad]
    float k_omega; // 旋回のフィードバックの角速度のゲイン[V/(rad/s)]
    float vd_max;  // |V_d| の上限[V]
    float kp;      // 並進の PI の比例ゲイン[V/(mm/s)](0 なら並進は FF だけ)
    float ki;      // 並進の PI の積分ゲイン[V/mm]
    float i_max;   // |ki·∫e dt| の上限[V]
    bool brake_test;       // 区間3 を一定の電圧のブレーキにする(E1B)
    float brake_v;         // ブレーキの電圧[V](負で逆向き)
    float brake_end_mm_s;  // 並進の速度がこれ未満になったらブレーキを終えて区間4 へ
    bool turn_test;        // 一定の区間の後に旋回のステップを入れる(E1T)
    float turn_dv;         // ステップの左右の差 ±ΔV[V](+ で左に曲がる向き)
    uint32_t turn_ms;      // ステップの長さの上限
    float turn_deg;        // ステップの間の回転がこれを越えたら終わる(0 なら見ない)
    bool turn_no_vd0;      // ステップで V_d0 を足さない(V_d = turn_dv。E1K0)
    bool cruise_theta_reset; // 一定の区間の始めに FB の θ を 0 に入れ直す(E1K0)
    uint8_t turn_seq_n;    // 2 以上ならステップを turn_seq_dv[] の順に turn_ms ずつ続ける(E1W。0・1 なら turn_dv の1段)
    float turn_seq_dv[FF_TURN_SEQ_MAX]; // 各段の V_d0 からの差[V]
    uint32_t vd_meas_ms;   // 一定の区間の最後のこの間の V_d を平均して v_d_meas にする
    float abort_dist_mm;   // |進んだ距離|(0 なら見ない)
    float abort_v_mm_s;    // |(vl + vr) / 2|(0 なら見ない)
    float abort_back_mm_s; // (vl + vr) / 2 < −これ で中止(0 なら見ない)
    float abort_angle_deg; // |積分角|(0 なら見ない)
    float abort_omega_dps; // |角速度|(0 なら見ない)
    bool abort_volt_over;  // |V_L| か |V_R| が電池の電圧を越えたら中止
    float abort_front_ir;  // 前の壁センサー(ad_fl・ad_fr の大きいほう)がこれを abort_front_ms 続けて越えたら中止(0 なら見ない)
    uint32_t abort_front_ms;
    uint16_t step_no;      // ログの step 列(何本目か)
} FfConfig;

typedef struct {
    FfConfig cfg;
    uint8_t seg;          // FfSegment
    bool done;
    uint8_t abort;        // StepAbort
    uint32_t seg_ticks;
    uint32_t still_ticks;
    float dt_s;
    float v_ref;          // [mm/s]
    float a_ref;          // [mm/s²]
    float v_cmd;          // 左右に分ける前の並進の電圧 V_s = v_ff + v_p + v_i[V]
    float v_ff;           // 並進の FF[V]
    float v_p, v_i;       // 並進の PI の比例・積分の出力[V](e = v_ref − v)
    float integ;          // ∫e dt[mm]
    float v_d;           // 旋回のフィードバックの出力 V_d[V](上限で止めた後)
    float theta_rad;      // フィードバックに使った角度 θ[rad]
    bool vd_sat;          // この tick に V_d を上限で止めた
    float theta_offset_deg; // θ = 積分角 − これ(E1T: 減速の始めに入れ直す)
    float turn_start_deg;   // E1T: ステップの始めの積分角
    float vd_sum;           // E1T: 一定の区間の最後の V_d の和
    uint32_t vd_count;
    float v_d_meas;         // E1T: 一定の区間の最後の V_d の平均(区間2 の後に決まる。それまで 0)
    float v_d0;             // E1T: ステップの V_d0(ステップの間だけ。ほかは 0)
    float dv_out;           // E1T: ステップの ±ΔV(ステップの間だけ。ほかは 0)
    uint8_t turn_idx;       // E1W: 今のステップの段(0 始まり)
    uint32_t front_ticks;   // 前の壁センサーがしきい値を越え続けている tick 数
    float v_left, v_right;
} FfTest;

void FfTest_Start(FfTest *ft, const FfConfig *cfg, float dt_s);

// ログの seg 列の値(E1T は 1 加速、2 一定、3 ステップ、4 減速、5 停止、9 中止。ほかは FfSegment のまま)
uint8_t FfTest_LogSeg(const FfTest *ft);

// 1 tick 進め、この tick にかける電圧を ft->v_left / v_right に入れる(done の後は 0V のまま)。
void FfTest_Update(FfTest *ft, const StepMeas *m);

#endif
