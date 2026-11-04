#ifndef INC_DRIVE_MODEL_H_
#define INC_DRIVE_MODEL_H_


#include "global.h"
#include "params.h"

// 機体モデル(docs/yuho_model.md)の逆で，目標から FF の電圧を出す．ハードに依存しない．
// 単位は SI(V，m/s，rad/s)．電圧は並進 V_s = (V_L + V_R)/2 と旋回 V_d = (V_R − V_L)/2(左回りが正)に分ける．
//   V_s,ff = a·v + V0·sign(v) + b·a_ref + c·v·ω²
//   V_d,ff = b_ω·ω̇ + D + F_ff                         (並進しながら曲がるとき．F_ff はタイヤが横にこする分)
//   V_d,ff = b_ω·ω̇ + D + 摩擦(向きごと) + a·(T/2)·ω   (超信地旋回．モデルの範囲の外なので仮の式)

typedef struct {
    float f_ff; // F の FF の状態[V]．曲がっていないときは 0 へ戻る
} DriveModelFF;

void DriveModel_Reset(DriveModelFF *m);

// 並進の FF[V]．v・a_ref・omega は目標(m/s，m/s²，rad/s)．
float DriveModel_FF_Vs(float v, float a_ref, float omega);

// 旋回の FF[V]．1 tick に 1 回だけ呼ぶ(F_ff を dt 進める)．
// v・omega・alpha は目標(m/s，rad/s，rad/s²)．
float DriveModel_FF_Vd(DriveModelFF *m, float v, float omega, float alpha, float dt);

#endif
