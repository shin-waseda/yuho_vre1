#ifndef INC_FAILSAFE_H_
#define INC_FAILSAFE_H_


#include "global.h"
#include "params.h"

// 同定のブランチでは目標速度がない(開ループ)ので、速度偏差の監視はしない。
// 番号は前と同じにしておく(LED の表示・ログの値を変えないため)。
typedef enum {
    FAILSAFE_NONE = 0,
    FAILSAFE_LOW_VOLTAGE,     // バッテリー電圧低下
    FAILSAFE_VELOCITY_DIVERGE, // (使っていない)
    FAILSAFE_GYRO_DIVERGE,    // 角速度の発散(スピン)
} FailSafeCause;

// 1kHzの監視に使う入力。SysidLoop_Tick()が毎tick詰めて渡す。
typedef struct {
    float battery_v;            // フィルタ前の電圧[V]
    float gyro_z_dps;           // 角速度Z[deg/s]
} FailSafeInput;

void FailSafe_Init(void);

// ISR(SysidLoop_Tick)から1kHzで呼ぶ。条件が継続したら発動してラッチする。
void FailSafe_Update(const FailSafeInput *in);

// 即座に発動させる(起動時の電圧チェックなど)。最初の原因だけが残る。
void FailSafe_Trip(FailSafeCause cause, float value);

bool FailSafe_IsTripped(void);
FailSafeCause FailSafe_GetCause(void);
const char *FailSafe_CauseName(FailSafeCause cause);

// フィルタ後のバッテリー電圧[V](表示用)。
float FailSafe_GetFilteredVoltage(void);

// メインループ用。発動原因をUARTへ出し、LEDを点滅させ続ける。戻らない。
// モーターの停止自体はISR側で済んでいる。
void FailSafe_Halt(void);

#endif
