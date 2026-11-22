#ifndef INC_DIAG_ROUTE_H_
#define INC_DIAG_ROUTE_H_


#include "global.h"
#include "params.h"
#include "logic/command.h"
#include "logic/maze/maze_types.h"
#include "logic/maze/wall_map.h"
#include "logic/maze/dijkstra.h"

// 斜めを使う最短走行の、区画の経路を求める向き付きの Dijkstra(2026-11-07)。
// dijkstra.h と同じコスト(1区画ごとに straight + 曲がり + 既知の区画)を 10 倍にし、次を変えたもの:
//   前の区画で右(左)へ曲がって入り、この区画で左(右)へ曲がって抜ける(ジグザグを続ける)ときは、
//   その区画の straight + turn90 の代わりに diag_cell_x10 を足す(0 なら変えない = dijkstra.h と同じ経路のコスト)。
// ジグザグは run_path の斜めの区間(DIAG)になるので、diag_cell_x10 を小さくすると斜めの多い経路を選ぶ。
// 10 倍にするのは、斜めの1区画を直進の1区画(10)より安く(道のりの比なら 7)できるようにするため。
// ノードは「区画pに向きdで入った状態」×「入るときの曲がり(なし・右・左)」(dijkstra.h の3倍)。
// 作業領域は全部 DiagRouteSolver の中に持つ(約 23KB)。
// キュー(2026-11-07): 1区画で増えるコストの最大が DIAG_ROUTE_BUCKETS − 1 以下ならバケツのキュー(Dial の方法。dijkstra.c と
// 同じ考え方)、そうでなければ二分ヒープ。どちらでも結果(コストと次の向き)は同じ。heap_only を true にすると二分ヒープだけ
// (計算時間を比べる用)。heap / heap_pos は、バケツのキューでは同じ箱の次・前のノードに使う。

#define DIAG_ROUTE_NODE_COUNT (MAZE_NODE_COUNT * 3)
#define DIAG_ROUTE_BUCKETS    512u // 2 のべき。既定のコスト(10 倍)で1区画の上乗せは最大 510

typedef struct {
    uint16_t cost[DIAG_ROUTE_NODE_COUNT];     // ゴールまでの最小コスト(MazeCost の 10 倍。行けなければ 0xFFFF)
    uint8_t next[DIAG_ROUTE_NODE_COUNT];      // 次の向き(bit0〜1)と、それが有効か(bit2)
    uint16_t heap[DIAG_ROUTE_NODE_COUNT];     // ノード番号の二分ヒープ(コストの小さい順)
    uint16_t heap_pos[DIAG_ROUTE_NODE_COUNT]; // ノード → ヒープ上の位置
    uint16_t heap_size;
    uint16_t bucket_head[DIAG_ROUTE_BUCKETS]; // 箱 → 先頭のノード(バケツのキュー)
    uint32_t bucket_cur;                      // 次に取り出すコストの下限
    bool use_bucket;                          // 今の計算でバケツのキューを使っているか
    bool heap_only;                           // 呼ぶ側が決める: true なら二分ヒープだけを使う(既定 false)
} DiagRouteSolver;

_Static_assert(DIAG_ROUTE_NODE_COUNT < 0xFFFFu, "node index must fit in uint16_t");

// goals のどれかの区画に入るまでのコストを、全てのノードについて求める。cost が NULL なら MazeCost_Default()。
void DiagRoute_Compute(DiagRouteSolver *s, const WallMap *map, WallView view, const MazeCost *cost,
                       uint16_t diag_cell_x10, const MazePos *goals, uint8_t goal_count);

// DiagRoute_Compute と同じだが、from に向き heading で止まっている(前の曲がりなし)ノードのコストが決まった所で止める
// (全面探索の最短経路の候補の計算用。from からの DiagRoute_Cost / BuildRoute / BuildDirs は DiagRoute_Compute と同じ結果。
// それより遠いノードの値は使わないこと)。
void DiagRoute_ComputeFrom(DiagRouteSolver *s, const WallMap *map, WallView view, const MazeCost *cost,
                           uint16_t diag_cell_x10, const MazePos *goals, uint8_t goal_count,
                           MazePos from, Direction heading);

// p に向き heading で止まっている(前の曲がりなし)ときの、ゴールまでのコスト(10 倍の値。行けなければ 0xFFFF)。
uint16_t DiagRoute_Cost(const DiagRouteSolver *s, MazePos p, Direction heading);

// start に向き heading で止まっている状態からゴールまでの経路を、指令の列(続く直進はまとめる)にして out へ書く。
// 最後は ACTION_STOP。行けなければ false。
bool DiagRoute_BuildRoute(const DiagRouteSolver *s, MazePos start, Direction heading, CommandList *out);

// DiagRoute_BuildRoute と同じ経路を、区画を移る向き(Direction)の列にして dirs へ最大 max 個書き、その数を返す
// (行けなければ 0。スタートがゴールでも 0)。
uint16_t DiagRoute_BuildDirs(const DiagRouteSolver *s, MazePos start, Direction heading, uint8_t *dirs, uint16_t max);

#endif
