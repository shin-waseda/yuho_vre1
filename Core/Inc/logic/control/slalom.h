#ifndef INC_SLALOM_H_
#define INC_SLALOM_H_


#include "global.h"
#include "params.h"

// スラローム(並進の速さ v を保ったまま、角速度を台形で動かして曲がる)の形を計算する。
// 制御の ISR と同じ台形(VelocityProfile、CONTROL_DT_S 刻み)を PC と同じように進めて、
// 曲がっている間に進む距離を求める。ハードに依存しない。

typedef struct {
    float v_mm_s;      // 並進の速さ
    float omega_dps;   // 最高角速度
    float alpha_dps2;  // 角加速度
    float angle_deg;   // 回る角度(正の値。向きは呼び出し側で決める)
} SlalomParams;

typedef struct {
    float forward_mm;     // 曲がる間に、曲がり始めの向きへ進む距離(曲がり終わりの位置)
    float side_mm;        // 曲がる間に、曲がる側へ進む距離(曲がり終わりの位置)
    float forward_max_mm; // 曲がる間に、曲がり始めの向きへ一番進んだ距離(180° で使う)
    float length_mm;      // 曲がる間に進む道のり(= v × 時間)
    float time_s;         // 曲がっている時間
} SlalomShape;

SlalomShape Slalom_ComputeShape(const SlalomParams *p);

// 曲がる形(軌跡)を変えずに並進の速さを v_mm_s にする: 角速度は速さに比例、角加速度は速さの2乗に比例させる。
// 前後のオフセットはほぼ同じになり(1tick 刻みの差だけ変わる)、横加速度は速さの2乗で増える。
void Slalom_ScaleToSpeed(SlalomParams *p, float v_mm_s);

// 90° 曲がる旋回の前後のオフセット。曲がり始めの向きに span_mm、曲がる側へ span_mm 移る旋回として
//   前のオフセット = span_mm − forward、後ろのオフセット = span_mm − side
// を返す(角速度の台形は左右対称なので、ふつうは同じ値になる)。負なら、その速さ・角速度では
// 収まらない(回り方が大きすぎる)。
//   小回り 90°(境目の真ん中 → 隣の境目の真ん中): span_mm = 半区画
//   大回り 90°(区画の中心 → 斜め隣の区画の中心): span_mm = 1区画
void Slalom_Turn90Offsets(const SlalomShape *s, float span_mm, float *pre_mm, float *post_mm);

// 大回り 180°(区画の中心 → 隣の列の区画の中心、柱を回る U ターン)用に、曲がる側へちょうど
// side_mm(= 1区画)移るような最高角速度を求めて p->omega_dps に書く(v と角加速度は p のまま)。
void Slalom_SolveOmegaForSide(SlalomParams *p, float side_mm);

// 大回り 180° の前後のオフセット。曲がり始めの向きに一番進んだ所が、出発した区画の中心から
// 1区画(cell_mm)先(柱の列の向こう側の区画の中心の高さ)になるよう前のオフセットを決め、
// 後ろのオフセットは出発した区画の中心の高さに戻るように決める。
void Slalom_Turn180Offsets(const SlalomShape *s, float cell_mm, float *pre_mm, float *post_mm);

// ---- スリップを含めた 90° の旋回(tools/matlab の yc.simulate_turn と同じ計算)----
// 機体の向き θ は台形どおりに回り、進む向きは θ − β(β はスリップアングル。+ で外へずれる)。
//   dβ/dt = (K × v[m/s] × ω[rad/s] − β) / C   (C = 0 なら β = K v ω)
// 前のオフセット → 曲がる → 後ろのオフセット → extra_mm 直進 を CONTROL_DT_S 刻みで進め、出口のずれを返す。
typedef struct {
    float along_mm;     // 後ろのオフセットの後、出る向きの前後のずれ(+ で先、− で遅れ)
    float lat_post_mm;  // 後ろのオフセットの後の横のずれ(外が +)
    float lat_final_mm; // extra_mm 進んだ後の横のずれ(スリップが収まった後)
} SlalomSlipError;

SlalomSlipError Slalom_SlipError90(const SlalomParams *p, float span_mm, float pre_mm, float post_mm,
                                   float extra_mm, float K, float C);

// 小回り 90°(探索・最短走行・SLALOM の試験で共通)の前後のオフセット。p は曲がる速さにしてあること
// (Slalom_ScaleToSpeed の後)。形で決まるオフセットに、次のモデルの調整を足す(速さごとの値は持たない):
//   スリップ(SLALOM_SLIP_K, SLALOM_SLIP_C_S): 出口のずれを Slalom_SlipError90 で求め、打ち消すように
//     前(外へのずれの分)と後ろ(前後のずれの分)を直す(tools/matlab の turn_sim の ADJ の提案と同じ。3回くり返す)
//   スリップ以外の遅れ(SLALOM_EXTRA_LAG_MM。速さによらず一定): 後ろを伸ばす
//   SLALOM_PRE_ADJ_MM / SLALOM_POST_ADJ_MM: さらに手で足す分(ふつうは 0)
typedef struct {
    float pre_mm;      // 前のオフセット(調整込み)
    float post_mm;     // 後ろのオフセット(調整込み)
    float pre_adj_mm;  // 形で決まる前のオフセットからの調整分(前壁補正の閾値の計算に使う)
    float post_adj_mm; // 形で決まる後ろのオフセットからの調整分
} SlalomOffsets;

SlalomOffsets Slalom_SmallTurnOffsets(const SlalomParams *p);

// 前壁補正の閾値(FL + FR)。曲がり始めが前の調整分だけ動くので、その位置で見える値にする。
float Slalom_FrontRefSum(float pre_adj_mm);

#endif
