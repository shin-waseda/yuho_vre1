#ifndef INC_MAZEPRINT_H_
#define INC_MAZEPRINT_H_


#include "global.h"
#include "params.h"
#include "logic/maze/maze_types.h"
#include "logic/maze/wall_map.h"
#include "logic/maze/dijkstra.h"

// 迷路をprintfで文字の絵にする(デバッグ用。実機ではUART、PCでは画面に出る)。
// 北が上。壁は既知の壁 "---" / '|'、未知の壁 " . " / ':'、壁なしは空白。
// 区画の中は 機体の向き(^ > v <)、ゴール(G)、solverを渡せばゴールまでのコスト
// (4つの向きのうち最小。行けなければ *、1000以上は ###)。
//
// solver・robotはNULLなら表示しない。
void MazePrint_Map(const WallMap *map, const MazeSolver *solver,
                   const MazePos *robot, Direction heading,
                   const MazePos *goals, uint8_t goal_count);

#endif
