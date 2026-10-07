#ifndef INC_WALLSENSE_H_
#define INC_WALLSENSE_H_


#include "global.h"
#include "params.h"
#include "logic/maze/wall_map.h" // WallObservation

// 壁センサーの値(IR 点灯 − 消灯の差)。ハードに依存しないよう、値だけを受け取る。
// 実機では app 層が ad_l / ad_fl / ad_fr / ad_r から作る。
typedef struct {
    uint16_t l;
    uint16_t fl;
    uint16_t fr;
    uint16_t r;
} WallSensorValues;

// 区画の境界で読んだ値から、前・右・左の壁の有無を判定する(しきい値は params.h の WALL_TH_*)。
// 結果は迷路のロジック(WallMap_Observe / SearchPlanner_Step)にそのまま渡せる。
WallObservation WallSense_Judge(WallSensorValues v);

#endif
