#ifndef INC_STEPMAP_H_
#define INC_STEPMAP_H_


#include "global.h"
#include "params.h"
#include "logic/maze/maze_types.h"
#include "logic/maze/wall_map.h"

// 足立法の歩数マップ(BlueEyesのmake_smap_adachiと同じ考え方)。
// ゴールを0とし、壁のない隣へ1ずつ増やした歩数を全区画に書く。向きや旋回は考えない。
// BlueEyesは歩数ごとに全区画を見直していたが、ここでは幅優先探索(キュー)で1回ずつ広げる。
// キュー(512B)は step_map.c に1つだけ置いて共有するので、同時に計算しないこと。
#define STEP_MAP_INF 0xFFFFu

// 計算結果(512B)
typedef struct {
    uint16_t step[MAZE_SIZE][MAZE_SIZE];  // [y][x] ゴールまでの歩数(行けなければINF)
} StepMap;

void StepMap_Compute(StepMap *s, const WallMap *map, WallView view,
                     const MazePos *goals, uint8_t goal_count);

// pに向きheadingでいるとき、歩数が1小さい隣の区画への向きをnextへ書く。
// 候補が複数あれば 前 → 右 → 左 → 後ろ の順で選ぶ(BlueEyesは北→東→南→西の順だった)。
// すでにゴールにいる・ゴールへ行けない場合はfalse。
bool StepMap_NextDir(const StepMap *s, const WallMap *map, WallView view,
                     MazePos p, Direction heading, Direction *next);

#endif
