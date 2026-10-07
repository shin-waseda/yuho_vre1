#ifndef INC_WALLCONTROL_H_
#define INC_WALLCONTROL_H_


#include "global.h"
#include "params.h"
#include "logic/wall_sense.h"

// 壁の制御の内部状態(左右それぞれ、切れ目の検出に使う)。
typedef struct {
    uint16_t last_value;   // 前に比べたときの値
    float last_dist_mm;    // 前に比べたときの距離
    float hold_until_mm;   // この距離までは使わない(切れ目・柱の横)
} WallSideState;

typedef struct {
    WallSideState left;
    WallSideState right;
    bool started;          // Reset 後、最初の Update で比べる基準を取ったか
} WallControl;

// 直進を始めるとき(または直進でなくなったとき)に呼ぶ。切れ目の検出の履歴を捨てる。
void WallControl_Reset(WallControl *wc);

// 1tick ぶん進める。v: 壁センサーの値、dist_mm: 進んだ距離(増えていく値)。
// 戻り値: 迷路の軸の向きに足す向きのオフセットの目標[deg](反時計回り正)。横のずれに比例し、
// ±WALL_OFFSET_MAX_DEG で頭打ち。使える壁がなければ 0(迷路の軸の向きに戻す)。
// used_left/used_right(NULL可): その tick で左右の壁を使ったか(ログ用)。
float WallControl_Update(WallControl *wc, WallSensorValues v, float dist_mm,
                         bool *used_left, bool *used_right);

#endif
