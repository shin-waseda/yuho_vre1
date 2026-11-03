#ifndef INC_SYSID_LOOP_H_
#define INC_SYSID_LOOP_H_

#include "global.h"
#include "params.h"
#include "logic/sysid/step_test.h"
#include "logic/sysid/ff_test.h"
#include "logic/state_estimation/imu.h"

// 同定のブランチの 1kHz の処理(前の app/control_loop の代わり。PID・向きや位置の制御はない)。
// 毎 tick: エンコーダ → 車輪の速度・進んだ距離、ジャイロ → 角速度・積分角、電池の電圧 → フェイルセーフ。
// 走っている間は logic/sysid/step_test(FF の検証では logic/sysid/ff_test)を1歩進め、
// duty = 電圧 / 電池の電圧(フィルタ後)で PWM にする(開ループ)。
// 0V は Motor_Drive(0, 0)(方向のピンはそのままで PWM 0。TB6612FNG ではショートブレーキのはず。未確認)。

// ICM_CalibrateBlocking() の後、HAL_TIM_Base_Start_IT(&htim6) の前に main() から1回呼ぶ。
void SysidLoop_Init(void);

// TIM6 の割り込み(interface/timer.c)から 1kHz で呼ぶ。
void SysidLoop_Tick(void);

// 1本走り始める(メインから呼ぶ。止まっているときだけ)。モーターのドライバを有効にし、次の tick で
// 進んだ距離・積分角を 0 にして、step_test と記録(Logger_Start)を同じ tick から始める。
// 記録の準備(Logger_Init・列・Logger_StreamBegin)は呼ぶ前に済ませておくこと。
// 記録の始め方は log で選ぶ(SysidLogStart)。
// 終わると(区間3・4・9 の後、またはフェイルセーフ)、割り込みが記録を止め(Logger_Stop)、モーターのドライバを無効にする。
typedef enum {
    SYSID_LOG_START = 0, // 走り始めの tick で Logger_Start
    SYSID_LOG_RESUME,    // 走り始めの tick で Logger_Resume(開いたままのファイルの続きに記録する)
    SYSID_LOG_KEEP,      // 記録はメインが走る前から始めている(E1S の走る前の1秒)。ここでは触らない
} SysidLogStart;
void SysidLoop_StartRun(const StepConfig *cfg, SysidLogStart log);
// SysidLoop_StartRun と同じだが、FF の検証(E1V。logic/sysid/ff_test)で1本走る。記録は Logger_Start から
void SysidLoop_StartFfRun(const FfConfig *cfg);
bool SysidLoop_IsDone(void);

typedef struct {
    bool failsafe;          // フェイルセーフで打ち切った
    uint8_t abort;          // StepAbort(0 なら中止していない)
    uint32_t ticks;         // 走り始めから終わりまでの tick 数
    uint32_t sat_ticks;     // 電圧が電池で出せる範囲を超え、PWM を上限で止めた tick 数(左右どちらか)
    uint32_t sat_first_tick; // 初めて上限で止めた tick(走り始めの tick が 0。E1V ではログの time_s と同じ数え方)。sat_ticks が 0 なら使わない
    uint32_t vd_sat_ticks;  // FF の検証: 旋回のフィードバック V_d を上限で止めた tick 数
    uint32_t vd_sat_first_tick; // 初めて V_d を上限で止めた tick(数え方は sat_first_tick と同じ)。vd_sat_ticks が 0 なら使わない
    float peak_v_mm_s;      // |並進の速度| の最大
    float peak_omega_dps;   // |角速度| の最大
    float dist_mm;          // 終わったときの進んだ距離
    float angle_deg;        // 終わったときの積分角
    float vbat_start;       // 走り始めの電池の電圧(フィルタ後)
    bool moved_l, moved_r;  // E0: 動き出したか
    float move_v_l, move_v_r; // E0: 動き出した電圧[V]
    uint16_t step;          // 終わったときのステップの番号(1 始まり。階段で中止したら、そのステップ)
    uint16_t steps;         // ステップの数
    float v_d_meas;         // E1T: 一定の区間の最後の V_d の平均[V]
} SysidResult;

// SysidLoop_IsDone() が true になった後に読む
SysidResult SysidLoop_GetResult(void);

// 6軸(ジャイロのゼロ点と、止まっているときの加速度)を ms ミリ秒の平均で測り直す(ブロッキング。止まっているときに呼ぶ)。
// 測った値(センサーの軸、生値[LSB])を返す。割り込みが動いていなければ前の値のまま。
// 待っている間も Logger_StreamPoll を呼ぶ(走る前から記録しているとき、ブロックを SD へ送るため)。
ImuCalib SysidLoop_RecalibrateImu(uint32_t ms);

float SysidLoop_GetVbat(void); // フィルタ後の電池の電圧[V]

// ログ用。毎 tick 更新する値(すべて float。Logger_AddField に各メンバのアドレスを渡す)
typedef struct {
    float step;     // ステップの番号(1 始まり。階段で何番目か。E1S は何番目の電圧か。走っていないときは 0)
    float seg;      // 区間の番号(StepSegment、FF の検証は FfTest_LogSeg。走っていないときは -1)
    float v_cmd;    // かけたい電圧(符号付き)[V]。旋回は正 = 反時計回り(右に +、左に −)。FF の検証は左右の補正の前の V
    float v_ref;    // FF の検証の目標の速さ[mm/s](ほかの試験では 0)
    float a_ref;    // FF の検証の目標の加速度[mm/s²](ほかの試験では 0)
    float v_s;      // FF の検証の並進の電圧 V_s = v_ff + v_p + v_i[V](v_cmd と同じ値)
    float v_ff;     // FF の検証の並進の FF[V]
    float v_p, v_i; // FF の検証の並進の PI の比例・積分の出力[V]
    float v_d;      // FF の検証の旋回のフィードバックの出力 V_d[V](+ で左に曲がる向き)
    float theta;    // FF の検証のフィードバックに使った角度 θ[rad](+ が左)
    float v_d0;     // E1T: ステップの V_d0[V](ステップの間だけ。ほかは 0)
    float dv;       // E1T: ステップの ±ΔV[V](ステップの間だけ。ほかは 0)
    float v_d_meas; // E1T: 一定の区間の最後の V_d の平均[V](区間2 の後に決まる。それまで 0)
    float pwm_l, pwm_r; // 実際に出した PWM の比較値(-PWM_MAX〜PWM_MAX)
    float vbat;     // 電池の電圧(フィルタ後、duty の計算に使った値)[V]
    float vl, vr;   // 車輪の速度[mm/s](前進が正)
    // 6軸(logic/state_estimation/imu の機体の座標: x = 前、y = 左、z = 上。x・y の向きは未確認)
    float gyro_z;   // 角速度[dps](反時計回りが正。ゼロ点を引いた後)
    float gyro_x, gyro_y;      // 角速度[dps](ゼロ点を引いた後)
    float acc_x, acc_y, acc_z; // 加速度[m/s²](重力を含む。止まっているときの値は走る前に UART に出す)
    float dist;     // 進んだ距離 (左 + 右) / 2 [mm](走り始めが 0)
    float angle;    // ジャイロの積分角[deg](走り始めが 0)
} SysidDebug;

const SysidDebug *SysidLoop_GetDebug(void);

#endif
