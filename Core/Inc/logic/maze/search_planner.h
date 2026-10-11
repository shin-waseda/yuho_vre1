#ifndef INC_SEARCHPLANNER_H_
#define INC_SEARCHPLANNER_H_


#include "global.h"
#include "params.h"
#include "logic/maze/maze_types.h"
#include "logic/maze/wall_map.h"
#include "logic/maze/dijkstra.h"
#include "logic/maze/step_map.h"
#include "logic/command.h"

// 探索の判断(ハードウェアに依存しない部分)。
// 実行する側(実機のapp層・PCのシミュレータ)は、区画に入るたびに
//   1. その区画で見えた壁を WallObservation にして SearchPlanner_Step() に渡す
//   2. 返ってきた Action を実行する
// を繰り返す。位置と向きはプランナーが自分で進めて覚えておく。
//
// スタートからゴール(どれかの区画)へ行き、着いたらSTOPを1回返して、
// 次の呼び出しからスタートへ戻る。スタートに着いたらSTOPを返して終わる。
// どちらも、未知の壁は「ない」とみなした地図で、毎回経路を計算し直す。
//
// Dijkstraのコストは行き(cost_to_goal)と帰り(cost_to_start)で別に持つ。
// 帰りは既知の区画のコストを上げて(known_cell)、まだ見ていない区画を通らせ、
// 最短経路の候補を増やす。Init後にメンバを書き換えれば変えられる(足立法では使わない)。
//
// 全面探索(SearchPlanner_StartFull): 最短経路になりうる区画だけを回り、最短経路が決まったらスタートへ戻る。
//   1. 未知の壁は「ない」とみなした地図で、スタートからゴールまでの最短経路(最短走行と同じ Dijkstra の既定のコスト)を求める。
//      未知の壁を「ない」とみなしているので、このコストは本当の最短のコストより大きくならない(下限)。
//   2. その経路が横切る壁のうち、まだ分かっていない壁の両側の区画(見ていない区画)を行き先にして、一番近いものへ向かう。
//   3. 経路が横切る壁が全部分かったら、その経路は実際に通れて、コストは下限と同じなので、本当の最短経路。
//      そこで探索をやめてスタートへ戻る(ゴールでは止まらない)。
//   見つかる壁は「ない」→「ある」に変わるだけなので、経路を計算し直すのは、経路が横切る壁が見つかったときだけでよい。
//   同じコストの経路が他にもあっても、そちらは見に行かない(最短走行が速くならないため)。
#define SEARCH_FULL_TARGET_MAX 255u // 全面探索の行き先の最大数(Dijkstra に渡す数が uint8_t のため)

typedef enum {
    SEARCH_ALGO_DIJKSTRA, // 向き付きDijkstra(旋回の少ない経路を選ぶ)
    SEARCH_ALGO_ADACHI,   // 足立法(歩数マップ)
} SearchAlgo;

typedef enum {
    SEARCH_PHASE_TO_GOAL,  // ゴールへ向かっている
    SEARCH_PHASE_TO_START, // スタートへ戻っている
    SEARCH_PHASE_DONE,     // スタートに戻った
    SEARCH_PHASE_FAILED,   // 分かっている壁でふさがれ、目的地へ行けない
    SEARCH_PHASE_FULL,     // 全面探索: 最短経路になりうる区画を回っている(終わったら TO_START)
} SearchPhase;

typedef struct {
    WallMap *map;      // 書き込む地図(呼び出し側が持つ。探索後の最短経路計算にも使う)
    SearchAlgo algo;
    SearchPhase phase;

    MazePos start;
    Direction start_heading; // スタートでの向き(全面探索の最短経路は、ここからゴールまでで計算する)
    MazePos goals[MAZE_GOAL_MAX];
    uint8_t goal_count;

    MazePos pos;       // 今いる区画
    Direction heading; // 今の向き

    MazeCost cost_to_goal;  // 既定: MazeCost_Default()
    MazeCost cost_to_start; // 既定: MazeCost_Default() + known_cell = MAZE_COST_KNOWN_CELL_RETURN
                            // (全面探索で行き先へ向かうときは cost_to_goal を使う)

    // 全面探索(SEARCH_PHASE_FULL)の最短経路の候補と行き先
    uint8_t full_route[MAZE_CELL_COUNT]; // スタートから進む向き(Direction)の列
    uint16_t full_route_len;
    bool full_route_valid;               // full_route を計算したか(StartFull で false に戻す)
    MazePos full_targets[SEARCH_FULL_TARGET_MAX]; // 見に行く区画
    uint8_t full_target_count;

    // 経路計算の結果。使うのはalgoで選んだ方だけなので、同じ場所に重ねて置く(2KB)。
    // 最後に計算した結果が残る。ただし Dijkstra は今いるノードのコストが決まった所で計算を止めるので
    // (Dijkstra_ComputeFrom)、それより遠いノードの値は入っていない(全部の値を表示するなら計算し直す)。
    union {
        MazeSolver solver; // SEARCH_ALGO_DIJKSTRA
        StepMap step;      // SEARCH_ALGO_ADACHI
    } work;
} SearchPlanner;

// mapはWallMap_Init()済み(または前回の探索の続き)であること。
// goal_countはMAZE_GOAL_MAXまで(超えた分は使わない)。
void SearchPlanner_Init(SearchPlanner *sp, WallMap *map, SearchAlgo algo,
                        MazePos start, Direction heading,
                        const MazePos *goals, uint8_t goal_count);

// 全面探索にする(Init の後、最初の Step の前に呼ぶ)。段階は SEARCH_PHASE_FULL になり、最短経路が決まったら
// (ゴールで止まらずに)TO_START へ移ってスタートへ戻る。
void SearchPlanner_StartFull(SearchPlanner *sp);

// 今いる区画で見えた壁を書き込み、次の指令を返す(位置・向きも指令の分だけ進める)。
// ゴール・スタートに着いたとき、目的地へ行けないとき、終わった後はACTION_STOP。
// 全面探索ではゴールで STOP を返さない(最短経路が決まった所から、そのままスタートへ向かう)。
Action SearchPlanner_Step(SearchPlanner *sp, WallObservation obs);

// 既知の区間をまとめて走るための先読み(機体の app/search_run と maze_sim で同じものを使う)。
// SearchPlanner_Step が直進(1区画)を返した直後に呼ぶ(sp->pos / heading は、その直進で入る区画 C1 と向き)。
// C1 から、プランナーの経路(Dijkstra の next_dir。今のノードより先はコストが小さいので、計算を途中で止めても
// 決まっている)をたどり、壁が全部分かっている区画が続く間の進む向きを moves[1..m] に書く(moves[0] は C1 に入る向き)。
// 目的地(全面探索では行き先)の区画・まだ分かっていない区画に入る所で止め、最後の2区画(終わりの区画の1つ手前と、終わりの区画に入る所)が
// まっすぐになるまで手前で切る(大回りが終わりの区画の真ん中で終わると、境界に戻れないため)。
// m < min_moves なら 0(まとめない)。Dijkstra でなければ 0。moves は MAZE_CELL_COUNT 個あること。
uint16_t SearchPlanner_KnownRun(const SearchPlanner *sp, uint16_t min_moves, Direction *moves);

#endif
