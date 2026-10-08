#ifndef INC_SENSORLOG_H_
#define INC_SENSORLOG_H_


#include "global.h"
#include "params.h"

// 壁センサーの値を測る試験(止まった状態)。モーターは動かさない。
// ボタン → 数秒ぶんの壁センサーの値を記録 → 平均・最小・最大を UART に表示
// → SD に保存(SDがあれば) → ボタン → PCへ送信、を繰り返す。
// 機体の置き方(壁のあり・なし、ずらす量など)を変えながら何回も測る想定。
// 電源を切るまで戻らない。
void SensorLog_Run(void);

#endif
