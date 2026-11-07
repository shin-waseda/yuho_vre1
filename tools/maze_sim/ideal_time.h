#ifndef IDEAL_TIME_H_
#define IDEAL_TIME_H_

// maze_sim の時間(C の CLI maze_sim.c と、mouse maze sim の窓口 yuho_mouse.c で共有する)。
// maze_sim はアルゴリズムの確認だけなので、時間は「計算上の速さ」の運動だけで数え、滑り・ログで合わせた補正・
// 尻当てや待ち・超信地旋回・flash に書く時間は数えない(時間で選ぶ経路などの、時間に依存するアルゴリズムを確かめるため)。
//   最短走行: logic 層の RunProfile_Default() の形(理想の円弧の長さ、オフセット 0、減速度 = 加速度)に速さを入れる
//   探索    : 指令ごとの運動を足す(t_s)
//     区画の境目は turn_v で通る(次の区画で曲がるかは、その区画に入るまで分からないため)。
//     直進    : 境目から境目まで台形加速(真ん中からなら止まった状態から半区画)
//     左右    : 小回り(半径 1/2 区画の 90°)を turn_v で。真ん中にいるときは、その場で回って(時間 0)半区画加速
//     180°    : 真ん中まで半区画で止まり、その場で回って(時間 0)半区画加速
//     止まる  : 真ん中まで半区画で止まる(ゴール・スタート)。ゴールでの 180° は時間 0
//     走ったままゴールに入る(往復でゴールを回る・全面探索): 最初のゴールの区画の真ん中まで半区画で止まり、真ん中から次の指令
//     既知の区間(app/search_run の TryKnownRun と同じ条件): 境目から境目まで、logic 層の SearchPlanner_KnownRun・
//               RunPath_FromKnownRun の指令の列を、理想の円弧の形で走る(入りと出は turn_v)

#include <stdbool.h>

#include "logic/command.h"
#include "logic/maze/search_planner.h"
#include "logic/maze/run_path.h"

// 探索の速さ
typedef struct {
    float v;      // 直進の最高速度 [mm/s]
    float turn_v; // 小回り(スラローム)の速さ [mm/s]。区画の境目はいつもこの速さで通る
    float accel;  // 加速度・減速度 [mm/s^2]
} SearchSpeeds;

// 探索の時間
typedef struct {
    SearchSpeeds sp;
    float t_s;      // ここまでの時間 [s]
    float t_goal_s; // 往復・片道: ゴールの真ん中に止まった時刻(ゴールを回るなら最初のゴールの区画)、
                    // 全面: 最短経路が決まった時刻(まだなら負)
    bool at_center; // 区画の真ん中で止まっている(そうでなければ境目を turn_v で通っている)
    int known_runs;  // 既知の区間をまとめて走った回数
    int known_cells; // まとめて走った区画の数(C1 から終わりの区画まで)
} SearchTime;

// params.h の SEARCH_V_MM_S, SLALOM_V_MM_S, SEARCH_ACCEL_MM_S2
SearchSpeeds SearchSpeeds_Default(void);
// "V,TURN_V,ACCEL"(どれも正)。読めなければ false(out は変えない)
bool SearchSpeeds_Parse(const char *text, SearchSpeeds *out);

// params.h の RUN_*(RunProfile_Default())
RunProfile IdealRun_Default(void);
// "VMAX,ACCEL,SMALL90,LARGE90,LARGE180"(どれも正)を RunProfile_Default() の形に入れる。読めなければ false
bool IdealRun_Parse(const char *text, RunProfile *out);

// スタートの区画の真ん中で止まった状態から
void SearchTime_Init(SearchTime *st, const SearchSpeeds *sp);

// 既知の区間をまとめて走る(app/search_run の TryKnownRun と同じ条件・同じ logic 層の計算)。
// SearchPlanner_Step が a を返した直後に、SearchTime_Step より先に呼ぶ(before / after は Step の前後の段階)。
// まとめるなら、その時間を足し、プランナーの位置・向きを区間の終わりの区画 E と入る向きへ進めて、
// moves[1..m](C1 から先に進む向き)の m を返す。このときは SearchTime_Step を呼ばない。
// まとめないなら 0 を返し、何もしない。moves は MAZE_CELL_COUNT 個あること
uint16_t SearchTime_KnownRun(SearchTime *st, SearchPlanner *sp, Action a, SearchPhase before, SearchPhase after,
                             Direction *moves);

// SearchPlanner_Step が返した指令 a の時間を足す。before / after は Step の前後の段階
void SearchTime_Step(SearchTime *st, Action a, SearchPhase before, SearchPhase after);

#endif
