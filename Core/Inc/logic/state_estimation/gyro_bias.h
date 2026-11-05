#ifndef INC_GYROBIAS_H_
#define INC_GYROBIAS_H_


#include "global.h"
#include "params.h"

// 走っている途中で機体が止まっている間に、ジャイロ Z のゼロ点を測り直す。ハードには触らない(値を受け取るだけ)。
// GYRO_STILL_WINDOW_MS ごとの窓で「止まっているか」を見る:
//   止まれの指令が窓の間ずっと出ている、エンコーダがほぼ動かない(左右とも GYRO_STILL_ENC_MAX_COUNTS 以下)、
//   ジャイロ Z と加速度の揺れ(標準偏差)が小さい、窓のジャイロの平均が今のゼロ点から離れすぎていない。
// 静かな窓が続いたら、最初の GYRO_STILL_SKIP_WINDOWS 個は捨て、その後の窓の平均へゼロ点を GYRO_BIAS_GAIN ずつ寄せる。
// 加速度は、エンコーダとジャイロが感じない車体の揺れ(止まった直後など)を見つけるのに使う。

typedef struct {
    // 今の窓の足し込み
    uint32_t n;
    float sum_g, sum_g2;           // ジャイロ Z の生値[LSB]
    float sum_a[3], sum_a2[3];     // 加速度[m/s²](機体の座標)
    int32_t enc_l, enc_r;          // エンコーダの変化の絶対値の合計[count]
    bool all_still;                // 窓の間ずっと止まれの指令だったか
    // 続けて静かだった窓
    uint32_t quiet_run;
    float run_start_offset;        // 続きの始めのゼロ点[LSB]
} GyroBiasEst;

typedef struct {
    float offset;          // 新しいゼロ点[LSB]
    bool run_ended;        // ゼロ点を寄せた続きが、今終わった(ログ用)
    uint32_t run_windows;  // その続きでゼロ点を寄せた窓の数
    float run_change;      // その間のゼロ点の変わり[LSB]
} GyroBiasStep;

void GyroBias_Init(GyroBiasEst *e);

// 1 tick ぶんを足す。gyro_raw: ジャイロ Z の生値[LSB]、acc: 加速度[m/s²](機体の x, y, z)、
// enc_l / enc_r: このtickのエンコーダの変化[count]、cmd_still: 止まれの指令(目標の速さ・角速度が 0 で、動きが終わっている)、
// offset: 今のゼロ点[LSB]。窓が終わったときだけゼロ点が変わることがある(それ以外は offset をそのまま返す)。
GyroBiasStep GyroBias_Update(GyroBiasEst *e, float gyro_raw, const float acc[3], int16_t enc_l, int16_t enc_r,
                             bool cmd_still, float offset);

#endif
