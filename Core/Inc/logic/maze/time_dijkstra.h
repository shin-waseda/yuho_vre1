#ifndef INC_TIME_DIJKSTRA_H_
#define INC_TIME_DIJKSTRA_H_


#include "global.h"
#include "params.h"
#include "logic/maze/maze_types.h"
#include "logic/maze/wall_map.h"
#include "logic/maze/run_path.h"

// 走行時間が最短になる最短走行の経路を求めるDijkstra(最短走行用)。
// 探索用の dijkstra.h(区画の中心がノード、コストは歩数と旋回の回数)とは別のもの。
//
// ノード: (区画, 向き, 旋回の種類)。「これからその種類の旋回を始める位置」を表す。
//   小回りは「区画に向きdで入る境目」、大回りは「区画の中心で向きd」から始まる
//   (run_path.h の指令の定義と同じ。番号の付け方は MazeNode_Index と同じ)。
//   旋回の種類は 小回り・大回り90°・大回り180°(TIME_CLASS_*)で、左右はどちらでもよい。
// 辺: 「旋回1つ + その後の直進 h 半区画(h ≥ 0)」をまとめて1本にする(run_path.h の指令2つ分)。
//   (位置, d, 種類a) → 旋回 → 直進 h 半区画 → (次の旋回を始める位置, 向き, 種類b)
//   直進は台形加速で a の速度 → b の速度(隣の旋回のオフセットと一緒に加減速する。
//   RunProfile_LinkedStraightTime)。h = 0 は旋回どうしが直接つながる場合で
//   (小回り → 小回り、大回り → 大回り。大回りどうしは区画を半分ずつ使い合う)、
//   前後のオフセットの中で a → b の速度に変えられるときだけ使える。
//   直進の後は必ず旋回(かゴール)なので、直進が2本続くことはない。
// スタートは区画の中心で静止、ゴールは最初に入ったゴールの区画の中心で静止。
// 直進や旋回がゴールの区画を通り抜けることはしない。
//
// ゴール側から逆向きに広げるので、全てのノードについて「そこからゴールまでの時間」が求まる。
// 計算用の優先度付きキュー(約12KB)は time_dijkstra.c に1つだけ置く。
// そのため TimeDijkstra_Compute() を同時に呼んではいけない。

enum {
    TIME_CLASS_SMALL90 = 0,
    TIME_CLASS_LARGE90,
    TIME_CLASS_LARGE180,
    TIME_CLASS_COUNT,
};

#define TIME_NODE_COUNT (MAZE_NODE_COUNT * TIME_CLASS_COUNT)
#define TIME_INF        0xFFFFFFFFu // 行けない

#define TIME_NEXT_STOP 0xFFu // TimeNext.next_class: 直進の後ゴールの中心で止まる

// ノードから次に進む辺(旋回 + 直進)。
typedef struct {
    uint8_t type;       // 旋回の RunType。0xFFなら次がない(行けない)
    uint8_t halves;     // 旋回の後の直進の長さ(半区画単位。0なら次の旋回へ直接つながる)
    uint8_t next_class; // 直進の後に始める旋回の種類。TIME_NEXT_STOP ならゴールで止まる
} TimeNext;

// 計算結果(約21KB)。
typedef struct {
    uint32_t time_us[TIME_NODE_COUNT]; // ゴールまでの時間[µs]
    TimeNext next[TIME_NODE_COUNT];

    // スタートから最初の指令(中心から北の境目へ出る直進)の選び方
    MazePos start;
    Direction start_heading;
    uint32_t start_time_us; // スタートからゴールまでの時間[µs](行けなければTIME_INF)
    uint8_t start_halves;   // 最初の直進の長さ(半区画単位)
    uint8_t start_class;    // 最初の直進の後の速度の種類
    bool start_to_goal;     // 最初の直進でそのままゴールに着く
} TimeSolver;

// 分かっている壁(viewで未知の壁の扱いを選ぶ。最短走行はWALL_VIEW_KNOWN)で、
// startに向きheadingで静止している状態から、goalsのどれかで止まるまでの最短時間を求める。
// profがNULLならRunProfile_Default()。
void TimeDijkstra_Compute(TimeSolver *s, const WallMap *map, WallView view, const RunProfile *prof,
                          const MazePos *goals, uint8_t goal_count,
                          MazePos start, Direction heading);

// スタートからゴールまでの時間[µs](行けなければTIME_INF)。
uint32_t TimeDijkstra_StartTime(const TimeSolver *s);

// 境目(区画pに向きdで入るところ)から、ゴールまでの時間[µs]の速度の種類での最小(表示用)。
uint32_t TimeDijkstra_NodeTime(const TimeSolver *s, MazePos p, Direction d);

// スタートからゴールまでの指令の列をoutへ書く(最後はRUN_STOP)。行けなければfalse。
bool TimeDijkstra_BuildRun(const TimeSolver *s, RunList *out);

#endif
