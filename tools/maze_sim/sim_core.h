#ifndef SIM_CORE_H_
#define SIM_CORE_H_

// tools/maze_sim の CLI (maze_sim.c) と GUI 用の DLL (sim_api.c) で共有する部分。
// 「本当の迷路」は WallMap で持ち、全部の壁を既知にしておく(WALL_VIEW_KNOWN で読む)。

#include "logic/command.h"
#include "logic/maze/maze_types.h"
#include "logic/maze/wall_map.h"

extern const MazePos kSimGoals[MAZE_GOAL_COUNT]; // params.h の MAZE_GOALS
extern const MazePos kSimStart;                  // params.h の MAZE_START_X/Y

// 迷路ファイル(micromouseonline/mazefiles の classic 形式)を読む。失敗したらfalse。
bool SimCore_LoadMazeFile(const char *path, WallMap *truth);

// 穴掘り法で迷路を作り、ところどころ壁を抜いてループも作る(seedが同じなら同じ迷路)。
void SimCore_MakeRandomMaze(uint32_t seed, WallMap *truth);

// pに向きheadingでいるときに見える前・右・左の壁
WallObservation SimCore_Sense(const WallMap *truth, MazePos p, Direction heading);

// 指令を本当の迷路で実行する(pos/headingを進める)。壁にぶつかったらfalse。
bool SimCore_Execute(const WallMap *truth, MazePos *pos, Direction *heading, Action a);

#endif
