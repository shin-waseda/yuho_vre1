#ifndef INC_STEP_TEST_H_
#define INC_STEP_TEST_H_

#include "global.h"
#include "params.h"

// 同定の実験(E0/E1/E2)の区間を進める状態機械。ハードには触らない(測った値を受け取り、かける電圧を返すだけ)。
// 1 tick ごとに StepTest_Update を呼び、返した電圧をその tick のモーターにかける。
//
// 区間(ログの seg 列):
//   0 始まり   0V を SYSID_PRE_MS(記録の始まりの目印)
//   1 +V      E1/E2: +V を t_acc_ms。E0: 0V から SYSID_E0_RAMP_V_PER_S で上げ、左右の車輪が両方動き出すまで
//   2 −V      E1/E2 だけ。速度(旋回は角速度)が 0 を切るまで(SYSID_REVERSE_TIMEOUT_MS を過ぎたら中止)
//   3 ブレーキ  0V を SYSID_BRAKE_MS(no_reverse のときは区間1の後すぐここへ来て、止まるまで。
//             止まった = |角速度|(並進は |速度|)< SYSID_STOP_* で、左右の車輪がどちらも SYSID_STOP_COUNTS カウント/tick 以下、
//             が SYSID_STOP_HOLD_MS 続いた。SYSID_STOP_TIMEOUT_MS で中止)
//   4 休み     0V を SYSID_REST_MS(階段で、次のステップの前。止まった状態から次をかける。
//             rest_after なら最後のステップの後にも休んでから done)
//   9 中止     0V を SYSID_BRAKE_MS(距離・角速度が上限を超えた、区間2で止まらなかった)。
//             no_reverse のときは区間3と同じく止まるまで(SYSID_STOP_TIMEOUT_MS で打ち切る)
// 区間3 の後、まだステップが残っていれば 4 → 1 と次へ進み、残っていなければ done。区間9 の後は残りを捨てて done。
//
// 階段(levels > 1): 電圧を volt, volt + v_inc, ... と levels 段上げる。alternate なら電圧ごとに
// sign の向き → 逆の向き の2ステップにする(ステップの数は levels × 2)。
//
// 電圧の向き: v_cmd は符号付き。並進は左右とも v_cmd(正 = 前進)。
// 旋回は右に +v_cmd、左に −v_cmd(正 = 反時計回り = 左回り。ジャイロの正と同じ)。

typedef enum {
    STEP_SEG_PRE = 0,
    STEP_SEG_ACC = 1,
    STEP_SEG_REV = 2,
    STEP_SEG_BRAKE = 3,
    STEP_SEG_REST = 4,
    STEP_SEG_ABORT = 9,
} StepSegment;

typedef enum {
    STEP_TEST_RAMP = 0, // E0
    STEP_TEST_STEP = 1, // E1(並進)・E2(旋回)
} StepTestType;

typedef enum {
    STEP_ABORT_NONE = 0,
    STEP_ABORT_DIST = 1,    // 並進で |進んだ距離| > SYSID_ABORT_DIST_MM
    STEP_ABORT_OMEGA = 2,   // 旋回で |角速度| > SYSID_ABORT_OMEGA_DPS
    STEP_ABORT_REVERSE = 3, // 区間2が SYSID_REVERSE_TIMEOUT_MS を過ぎても止まらなかった
    STEP_ABORT_NO_STOP = 4, // no_reverse の区間3が SYSID_STOP_TIMEOUT_MS を過ぎても止まらなかった
    STEP_ABORT_WHEEL = 5,   // 左右どちらかの |車輪の速さ| > SYSID_ABORT_WHEEL_MM_S
    STEP_ABORT_SPEED = 6,   // 並進で |並進の速度| > abort_v_mm_s
    STEP_ABORT_ANGLE = 7,   // 並進で |積分角| > abort_angle_deg
    STEP_ABORT_BACKWARD = 8, // ブレーキの試験(E1B)で並進の速度 < −abort_back_mm_s(後ろに戻った)
    STEP_ABORT_VOLT = 9,     // FF の試験で |V_L| か |V_R| が電池の電圧を越えた(E1T)
    STEP_ABORT_OBSTACLE = 10, // FF の試験で前の壁センサーが障害物を見た(E1T)
} StepAbort;

typedef struct {
    StepTestType type;
    bool rotate;       // false: 並進(E0・E1)、true: 超信地旋回(E0・E2)
    float sign;        // +1: 前進・左回り、-1: 後退・右回り(階段の alternate では、各電圧の1つ目の向き)
    float volt;        // E1/E2 でかける電圧の大きさ[V](E0 では使わない。階段では1段目)
    uint32_t t_acc_ms; // E1/E2 の区間1の長さ[ms]
    uint8_t levels;    // 電圧の段の数(1 なら1ステップだけ。0 も 1 として扱う)
    float v_inc;       // 階段で1段ごとに上げる電圧[V]
    bool alternate;    // 階段で、電圧ごとに左右(前後)の2ステップにする
    bool no_reverse;   // 区間2(−V)を使わず、区間1の後は 0V で止まるのを待つ
    // 並進の中止の上限(0 なら距離は SYSID_ABORT_DIST_MM、速度・角度は見ない)。E1S で使う
    bool no_dist_abort; // 並進で距離を見ない(浮かせて空転させる E1S。車輪の速さの上限は見る)
    float abort_dist_mm;
    float abort_v_mm_s;
    float abort_angle_deg;
    uint16_t step_base; // ログの step 列に足す値(1本1ステップの E1S で、何番目の電圧かを出す)
    bool skip_pre;      // 区間0 を飛ばして区間1 から始める(E1S で、同じファイルの2本目から。区間0 はファイルの始まりの目印)
    bool rest_after;    // 最後のステップの後にも区間4 で休んでから終わる(E1S)
} StepConfig;

// 1 tick ごとの測った値(走り始めを 0 にした値)
typedef struct {
    float dist_l_mm;   // 左の車輪が進んだ距離(前進が正)
    float dist_r_mm;   // 右の車輪が進んだ距離(前進が正)
    float vl_mm_s;     // 左の車輪の速さ(この tick のカウントの差 × MM_PER_PULSE / dt)
    float vr_mm_s;     // 右の車輪の速さ
    float v_mm_s;      // 並進の速度 (vl + vr) / 2
    float omega_dps;   // ジャイロの角速度(反時計回りが正)
    float angle_deg;   // ジャイロの積分角(走り始めが 0)
    float vbat_v;      // 電池の電圧(フィルタ後)[V](FF の試験の中止に使う)
    float front_ir;    // 前の壁センサーの値(ad_fl・ad_fr の大きいほう。FF の試験の中止に使う)
} StepMeas;

typedef struct {
    StepConfig cfg;
    uint8_t seg;          // StepSegment
    bool done;
    uint8_t abort;        // StepAbort
    uint32_t seg_ticks;   // 今の区間に入ってからの tick 数
    uint32_t still_ticks; // no_reverse の区間3で、止まった状態が続いている tick 数
    float dt_s;
    uint16_t step;        // 今のステップの番号(0 始まり)
    uint16_t steps;       // ステップの数
    bool final_rest;      // 今の区間4 は最後のステップの後の休み(終わったら done)
    float cur_volt;     // 今のステップの電圧の大きさ[V]
    float cur_sign;       // 今のステップの向き
    float v_cmd;          // この tick にかける電圧(符号付き)[V]
    float v_left, v_right;
    // E0: 左右の車輪が SYSID_E0_MOVE_MM 動いたときにかけていた電圧の大きさ[V](動いていなければ moved_* が false)
    bool moved_l, moved_r;
    float move_v_l, move_v_r;
} StepTest;

void StepTest_Start(StepTest *st, const StepConfig *cfg, float dt_s);

// 1 tick 進め、この tick にかける電圧を st->v_left / v_right に入れる(done の後は 0V のまま)。
void StepTest_Update(StepTest *st, const StepMeas *m);

#endif
