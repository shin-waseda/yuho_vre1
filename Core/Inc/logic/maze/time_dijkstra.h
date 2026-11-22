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
// ノード: (位置, 旋回の種類)。「これからその種類の旋回を始める位置」を表す(run_path.h の指令の定義と同じ)。
//   直交の位置: 小回り・斜めの入り(D45IN・D135IN)は「区画に向きdで入る境目」、大回りは「区画の中心で向きd」。
//   斜めの点(2026-11-07): 「区画に向きdで入る境目を、ジグザグの続きの向き z へ傾いて通る所」。
//     そこから始まる旋回は 斜めの出(D45OUT・D135OUT)・V90・柱を回る V90。
//   旋回の種類(TIME_CLASS_*)は左右を区別しない(斜めの点では z で左右が決まる)。
// 辺: 「旋回1つ + その後の直進 h(h ≥ 0)」をまとめて1本にする(run_path.h の指令2つ分)。
//   直交の位置で終わる旋回の後は直進(半区画単位)、斜めの点で終わる旋回の後は斜めの直進(DIAG。1区画/√2 単位)。
//   直進は台形加速で前の旋回の速度 → 次の旋回の速度(隣の旋回のオフセットと一緒に加減速する。
//   RunProfile_LinkedStraightTime)。h = 0 は旋回どうしが直接つながる場合で、
//   前後のオフセットの中で速度を変えられるときだけ使える。直進の後は必ず旋回(かゴール)。
// スタートは区画の中心で静止、ゴールは最初に入ったゴールの区画の中心で静止(斜めのままゴールには入らない)。
// 直進や旋回がゴールの区画を通り抜けることはしない。
//
// ゴール側から逆向きに広げるので、全てのノードについて「そこからゴールまでの時間」が求まる。
// 作業領域(ヒープ・直進の時間の表)も TimeSolver の中に持つ(全部で約 170KB。maze_sim だけで使う)。

enum {
    // 直交の位置から始まる旋回
    TIME_CLASS_SMALL90 = 0,
    TIME_CLASS_LARGE90,
    TIME_CLASS_LARGE180,
    TIME_CLASS_D45IN,
    TIME_CLASS_D135IN,
    TIME_CLASS_ORTH_COUNT,
    // 斜めの点から始まる旋回
    TIME_CLASS_D45OUT = TIME_CLASS_ORTH_COUNT,
    TIME_CLASS_D135OUT,
    TIME_CLASS_DV90,
    TIME_CLASS_DV90P,
    TIME_CLASS_COUNT,
};
#define TIME_CLASS_DIAG_COUNT (TIME_CLASS_COUNT - TIME_CLASS_ORTH_COUNT)

// ノードの番号: 直交の位置は MazeNode_Index × 直交の種類 + 種類、斜めの点はその後ろに (MazeNode_Index × 2 + 傾き) × 斜めの種類 + 種類
#define TIME_ORTH_NODE_COUNT (MAZE_NODE_COUNT * TIME_CLASS_ORTH_COUNT)
#define TIME_NODE_COUNT      (TIME_ORTH_NODE_COUNT + MAZE_NODE_COUNT * 2 * TIME_CLASS_DIAG_COUNT)
#define TIME_INF             0xFFFFFFFFu // 行けない

#define TIME_NEXT_STOP 0xFFu // TimeNext.next_class: 直進の後ゴールの中心で止まる

#define TIME_HALVES_MAX  (2 * MAZE_SIZE + 1)    // 直進の長さの上限(半区画、斜めは区間の数)
#define TIME_SPEED_COUNT (TIME_CLASS_COUNT + 1) // 速度の添字(旋回の種類 + 静止)

// ノードから次に進む辺(旋回 + 直進)。
typedef struct {
    uint8_t type;       // 旋回の RunType。0xFFなら次がない(行けない)
    uint8_t halves;     // 旋回の後の直進の長さ(半区画、斜めの点で終わる旋回なら斜めの区間の数。0なら次の旋回へ直接つながる)
    uint8_t next_class; // 直進の後に始める旋回の種類。TIME_NEXT_STOP ならゴールで止まる
} TimeNext;

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

    // ---- 計算の作業領域 ----
    bool use_diag;
    uint32_t straight_us[TIME_HALVES_MAX + 1][TIME_SPEED_COUNT][TIME_SPEED_COUNT]; // 直進の時間(行けなければTIME_INF)
    uint32_t diag_us[TIME_HALVES_MAX + 1][TIME_SPEED_COUNT][TIME_SPEED_COUNT];     // 斜めの直進の時間
    uint32_t turn_us[RUN_TYPE_COUNT]; // 旋回の曲線の部分の時間(オフセットは直進の側に数える)
    uint16_t heap[TIME_NODE_COUNT];     // ノード番号の二分ヒープ(時間の小さい順)
    uint16_t heap_pos[TIME_NODE_COUNT]; // ノード → ヒープ上の位置
    uint16_t heap_size;
} TimeSolver;

// 分かっている壁(viewで未知の壁の扱いを選ぶ。最短走行はWALL_VIEW_KNOWN)で、
// startに向きheadingで静止している状態から、goalsのどれかで止まるまでの最短時間を求める(斜めは使わない)。
// profがNULLならRunProfile_Default()。
void TimeDijkstra_Compute(TimeSolver *s, const WallMap *map, WallView view, const RunProfile *prof,
                          const MazePos *goals, uint8_t goal_count,
                          MazePos start, Direction heading);

// TimeDijkstra_Compute の、斜め(D45・D135・V90・柱を回る V90 と斜めの直進)も使えるもの(use_diag が false なら同じ)。
void TimeDijkstra_ComputeEx(TimeSolver *s, const WallMap *map, WallView view, const RunProfile *prof,
                            const MazePos *goals, uint8_t goal_count,
                            MazePos start, Direction heading, bool use_diag);

// スタートからゴールまでの時間[µs](行けなければTIME_INF)。
uint32_t TimeDijkstra_StartTime(const TimeSolver *s);

// 境目(区画pに向きdで入るところ)から、ゴールまでの時間[µs]の直交の旋回の種類での最小(表示用)。
uint32_t TimeDijkstra_NodeTime(const TimeSolver *s, MazePos p, Direction d);

// スタートからゴールまでの指令の列をoutへ書く(最後はRUN_STOP)。行けなければfalse。
bool TimeDijkstra_BuildRun(const TimeSolver *s, RunList *out);

#endif
