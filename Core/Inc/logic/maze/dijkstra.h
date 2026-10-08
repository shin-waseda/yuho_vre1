#ifndef INC_DIJKSTRA_H_
#define INC_DIJKSTRA_H_


#include "global.h"
#include "params.h"
#include "logic/maze/maze_types.h"
#include "logic/maze/wall_map.h"
#include "logic/command.h"

// 向き付きのDijkstra(BlueEyesのdijkstra_multi_goalと同じ考え方)。
// ノードは「区画pに向きdで入った状態」。そこからゴールまでの最小コストを、
// ゴール側から逆向きに広げて求める。1区画進むごとに MazeCost のコストを足す。
// 機体の今の向きも含めて計算するので、旋回の少ない経路を選べる。
//
// 計算用の優先度付きキュー(4KB)は dijkstra.c に1つだけ置き、全てのMazeSolverで共有する。
// そのため Dijkstra_Compute() を同時に(ISRとメインから等)呼んではいけない。

// 1区画進むときのコスト = straight + (向きを変えたら turn90 / turn180)
//                        + (入る区画の壁が4方向とも分かっていれば known_cell)
// コストは13bit(MAZE_COST_INF未満)で持つので、経路のコストの合計がそれを超えないこと。
// 目安: MAZE_CELL_COUNT × (straight + turn90 + known_cell) + 2 × turn180 < MAZE_COST_INF
typedef struct {
    uint16_t straight;
    uint16_t turn90;
    uint16_t turn180;
    uint16_t known_cell; // 0なら既知・未知を区別しない
} MazeCost;

#define MAZE_COST_FITS(s, t90, t180, k) \
    ((uint32_t)MAZE_CELL_COUNT * ((s) + (t90) + (k)) + 2u * (t180) < MAZE_COST_INF)

// params.h の MAZE_COST_STRAIGHT / TURN90 / TURN180、known_cell = 0(最短走行用)。
MazeCost MazeCost_Default(void);

// 計算結果(2KB)。ノードの状態だけを持つ。
typedef struct {
    MazeNodeState node[MAZE_NODE_COUNT];
} MazeSolver;

// goalsのどれかの区画に入るまでのコストを、全ての(区画, 向き)について求める。
// viewで未知の壁の扱いを選ぶ(探索中はSEARCH、最短走行はKNOWN)。
// costがNULLならMazeCost_Default()。
void Dijkstra_Compute(MazeSolver *s, const WallMap *map, WallView view, const MazeCost *cost,
                      const MazePos *goals, uint8_t goal_count);

// Dijkstra_Compute と同じだが、fromに向きheadingでいるノードのコストが決まった所で止める(探索の1歩ごとの計算用)。
// fromからのDijkstra_Cost / Dijkstra_NextDir / Dijkstra_BuildRoute は Dijkstra_Compute と同じ結果になる。
// それより遠い(コストの大きい)ノードは計算されていないので、他のノードの値は使わないこと。
void Dijkstra_ComputeFrom(MazeSolver *s, const WallMap *map, WallView view, const MazeCost *cost,
                          const MazePos *goals, uint8_t goal_count, MazePos from, Direction heading);

// pに向きheadingでいるときの、ゴールまでのコスト(行けなければMAZE_COST_INF)。
uint16_t Dijkstra_Cost(const MazeSolver *s, MazePos p, Direction heading);

// pに向きheadingでいるとき、次に進む向き(絶対方位)をnextへ書く。
// すでにゴールにいる・ゴールへ行けない場合はfalse。
bool Dijkstra_NextDir(const MazeSolver *s, MazePos p, Direction heading, Direction *next);

// startに向きheadingでいる状態からゴールまでの経路を、指令の列にしてoutへ書く。
// 最後は必ずACTION_STOP。merge_forwardがtrueなら、続く直進を1つの指令にまとめる。
// ゴールへ行けなければfalse(outは空)。
bool Dijkstra_BuildRoute(const MazeSolver *s, MazePos start, Direction heading,
                         bool merge_forward, CommandList *out);

#endif
