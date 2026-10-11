#ifndef SEARCH_TIME_H_
#define SEARCH_TIME_H_

// 探索の走行時間の見積もり(maze_sim 用。ファームウェアは使わない)。
// 実機の探索(app/search_run.c の SearchLoop)が指令ごとにする動きを、params.h の値と走る前に選ぶ速さでたどって時間を足す。
//   直進      : 境界から境界まで、スラロームの速さで入って出る(間で直進の速さまで加速)。真ん中からなら止まった状態から
//   左右      : スラローム(境界から境界、スラロームの速さのまま)。超信地旋回を選んだとき・真ん中にいるときは、
//               真ん中で止まる → 超信地旋回 → 半区画加速
//   180°      : 真ん中で止まる → (前と横に壁があれば 90° と尻当てを2回、なければその場で 180°)→ 半区画加速
//   ゴール    : 真ん中で止まる → TurnBackAtGoal(壁があれば尻当て)→ SEARCH_GOAL_WAIT_MS 待つ(片道はゴールで待って終わり)
//   既知の区間: 先の区画の壁が分かっていれば、まとめて最短走行の指令の列で走る(TryKnownRun)
//   スタート  : 右90° → 尻当て → 左90° → 尻当て(StartSequence)
// 数えないもの: 手を離してから走り出すまでの待ち、区画ごとの計算(走りながら行う)、前壁補正・壁切れ補正による位置のずれ。
// 2026-10-10 の探索のログ(search_*)で、指令ごとの時間を確かめた(development_log.md 12章)。
//
// 使い方: 走る前に SearchTime_Init と SearchTime_Start。SearchPlanner_Step のたびに SearchTime_Step(maze_sim のように
// 1区画ずつ進めてよい。既知の区間としてまとめて走る所は、SearchTime_Step が先読みして、その後の区画の分を数えない)。

#include <stdbool.h>
#include <stdint.h>

#include "logic/command.h"
#include "logic/maze/wall_map.h"
#include "logic/maze/search_planner.h"
#include "logic/maze/run_path.h"

// 探索の行き先(機体の RUN の SEARCH の SCOPE と同じ番号)
typedef enum {
    SEARCH_TIME_ROUND = 1,   // 往復
    SEARCH_TIME_ONE_WAY = 2, // 片道(ゴールで止まって終わる)
    SEARCH_TIME_FULL = 3,    // 全面(SearchPlanner_StartFull)
} SearchTimeScope;

typedef struct {
    float v_mm_s;      // 直進の最高速度(機体の SPEED)
    float turn_v_mm_s; // スラロームの速さ(機体の SLALOM。境界はいつもこの速さで通る)
    float accel_mm_s2; // 直進の加速度・減速度(機体の ACCEL)
    bool slalom;       // 曲がるときスラローム(true)か超信地旋回(false)か(機体の TURN)
    SearchTimeScope scope;
} SearchTimeParams;

typedef struct {
    SearchTimeParams p;
    float t_s;          // ここまでの時間[s](スタートの StartSequence の始めから)
    float t_goal_s;     // 往復・片道: ゴールの真ん中に止まった時刻、全面: 最短経路が決まった時刻(まだなら負)
    bool at_center;     // 区画の真ん中で止まっている(そうでなければ境界をスラロームの速さで通っている)
    bool finished;      // 探索が終わった(スタートに戻って 180° 回った・片道でゴールに止まった・失敗)
    uint16_t skip;      // この後、既知の区間としてまとめて走った分として数えない Step の数

    // 内訳[s]
    float t_straight;   // 直進(境界から境界、真ん中からの半区画加速、真ん中で止まる)
    float t_slalom;     // スラローム
    float t_pivot;      // 超信地旋回(前後の待ちを含む)
    float t_setpos;     // 尻当て
    float t_goal_wait;  // ゴール・スタートで止まっている時間
    float t_known;      // 既知の区間をまとめて走った時間
    uint16_t n_setpos;
    uint16_t n_known;        // 既知の区間をまとめて走った回数
    uint16_t n_known_cells;  // まとめて走った区画の数

    // Init で計算するもの
    float slalom_s;      // スラローム1回(境界から境界)の時間
    RunProfile known_prof; // 既知の区間を走るときの速さ(機体の ComputeSlalomOffsets と同じ)
    bool known_large;      // 既知の区間で大回りを使うか
} SearchTime;

// 機体の既定の探索の設定(params.h の SEARCH_V_MM_S、SLALOM_V_MM_S、SEARCH_ACCEL_MM_S2、スラローム、往復)
SearchTimeParams SearchTime_DefaultParams(void);

void SearchTime_Init(SearchTime *st, const SearchTimeParams *p);

// スタートの StartSequence(右90° → 尻当て → 左90° → 尻当て)。
void SearchTime_Start(SearchTime *st);

// SearchPlanner_Step の直後に呼ぶ。obs はその Step に渡した壁(今いる区画の、Step の前の向きから見た前・右・左)、
// a は返ってきた指令。sp は Step の後のプランナー(既知の区間の先読みと段階を見る)。
// ゴールで止まった後(段階が TO_START)、機体はその場で 180° 回っている。maze_sim も同じにすること
// (プランナーの向きを逆にする。SearchTime はそれを前提に、次の指令を真ん中から数える)。
void SearchTime_Step(SearchTime *st, const SearchPlanner *sp, WallObservation obs, Action a, SearchPhase phase_before);

// 1つの指令の時間を足す(ログの再生用。SearchTime_Step はこれを使う)。
// known が NULL でなければ、直進の代わりにその指令の列(RunPath_FromKnownRun の結果)で既知の区間を走った。
// phase_after は指令を返した後のプランナーの段階、walls は今いる区画の壁(向きは指令を返す前のもの)。
void SearchTime_Action(SearchTime *st, Action a, WallObservation walls, SearchPhase phase_after, const RunList *known,
                       uint16_t known_cells);

// 既知の区間をまとめて走る時間[s](境界をスラロームの速さで通ってから、終わりの区画の入口の境界まで)
float SearchTime_KnownRunTime(const SearchTime *st, const RunList *list);

#endif
