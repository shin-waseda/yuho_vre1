#ifndef INC_RUN_PATH_H_
#define INC_RUN_PATH_H_


#include "global.h"
#include "params.h"
#include "logic/command.h"
#include "logic/maze/maze_types.h"

// 最短走行の指令(走行パターン)と、その走行時間の見積もり。
// 探索の指令(command.h の Action)は「隣の区画へ移る」単位だが、こちらは実際の動きの単位。
//
// 位置の基準: 指令の切れ目は「区画の境目の真ん中」か「区画の中心」。
//   直進は半区画単位で持つ。中心 → 境目 が 1、境目 → 次の境目が 2。
//   小回りは境目から境目まで、大回りは区画の中心から区画の中心まで(下の表)。
//   斜めを足すときも、境目の真ん中どうしをつなぐ指令として加える予定。
//
// 区画の経路で見ると、各区画で「まっすぐ抜ける(0)」か「右(R)・左(L)へ曲がる」かが決まる。
// 旋回の指令は、その並びのうち次の形の部分を置き換える(右旋回の場合)。
//
// | 指令      | 区画ごとの曲がり方 | 動き                                                       |
// |-----------|--------------------|------------------------------------------------------------|
// | SMALL90   | R                  | 境目 → 同じ区画の右の境目(半径 1/2 区画)                   |
// | LARGE90   | 0 R 0              | 区画Aの中心 → Cの中心(半径1区画の90°)                     |
// | LARGE180  | 0 R R 0            | 区画Aの中心 → 隣の列のDの中心(柱を回るUターン)           |
//
// 大回りは「半区画・曲がる区画・半区画」を1つの指令にしたもの。0 の区画の残りの半区画は
// 前後の直進に入る(例: 1区画直進 → 0 R 0 → 1区画直進 は STRAIGHT 3, LARGE90_R, STRAIGHT 3)。
// 0 R 0 R 0(2列ずれるUターン)は、区画を共有する大回り90°の2連続(LARGE90_R, LARGE90_R)になる。
// 小回りの直後に大回りが来るときは、間に半区画の直進(STRAIGHT 1)が入る。
typedef enum {
    RUN_STOP = 0,     // 止まる(経路の終わり)
    RUN_STRAIGHT,     // halves × 半区画 まっすぐ進む
    RUN_SMALL90_R,
    RUN_SMALL90_L,
    RUN_LARGE90_R,
    RUN_LARGE90_L,
    RUN_LARGE180_R,
    RUN_LARGE180_L,
    RUN_TYPE_COUNT,
} RunType;

// 2バイト。typeはRunTypeの値。
typedef struct {
    uint8_t type;   // RunType
    uint8_t halves; // 直進の長さ(半区画単位、1以上)。旋回とSTOPは0
} RunCommand;

// 1区画ごとに曲がっても入る長さ + スタートとゴールの半区画 + 終端のSTOP
#define RUN_LIST_MAX (MAZE_SIZE * MAZE_SIZE + 3)

typedef struct {
    RunCommand items[RUN_LIST_MAX];
    uint16_t count;
} RunList;

// 速度のパラメータ。RunProfile_Default() は params.h の RUN_* の値。
// 旋回の速度・長さ・オフセットは [RunType] で引く(直進・STOPの欄は使わない)。
//
// 旋回は 前オフセット(直線) → 曲線 → 後オフセット(直線) の形で、指令の始めから終わりまで
// (小回りは境目から境目、大回りは中心から中心)の長さが turn_len。
// 時間の数え方(曲線の部分だけが一定の速度で、オフセットは直進と一緒に加減速する):
//   - 旋回の時間は、曲線の部分(turn_len − 前後オフセット)を旋回の速度で走る時間
//     (RunProfile_CurveTime)。
//   - 直進の時間は、隣の旋回のオフセットを含めた長い直進の時間(RunProfile_LinkedStraightTime)。
//   - 旋回どうしが直接つながるときは、前の旋回の後オフセット + 次の旋回の前オフセットを
//     長さ0の直進とみなして同じように扱う(速度が違えば、オフセットの中で変える)。
//   オフセットを旋回の側に数えて直進から引く分け方だと、直結の区間で加速した分が
//   負の時間になり、Dijkstra で扱えない。この分け方なら、どの部分の時間も0以上になる。
// 既定値は円弧だけの理想的な形(長さは RunType_TurnLength()、オフセットは0)。
// 実機でスラロームの前後オフセットを入れたら、turn_len と turn_pre / turn_post を合わせて変える。
typedef struct {
    float accel;                     // [mm/s²] 直進の加速度・減速度
    float vmax;                      // [mm/s] 直進の最高速度
    float v_turn[RUN_TYPE_COUNT];    // [mm/s] 旋回の種類ごとの速度
    float turn_len[RUN_TYPE_COUNT];  // [mm] 旋回の経路の長さ(オフセットを含む)
    float turn_pre[RUN_TYPE_COUNT];  // [mm] 前オフセット
    float turn_post[RUN_TYPE_COUNT]; // [mm] 後オフセット
} RunProfile;

RunProfile RunProfile_Default(void);

// 旋回ならtrue
bool RunType_IsTurn(RunType type);

// 旋回の経路の長さ[mm](理想的な円弧。小回りは境目から境目、大回りは中心から中心)。
// 旋回でなければ0。
float RunType_TurnLength(RunType type);

// 大回り(中心から中心へ曲がる旋回)ならtrue
bool RunType_IsLarge(RunType type);

// 旋回の曲線の部分(オフセットを除く)を旋回の速度で走る時間[s]。旋回でなければ0。
float RunProfile_CurveTime(const RunProfile *prof, RunType type);

// 旋回の向きの変化(90°単位、時計回りが正)。直進・STOPは0。
int RunType_QuarterTurns(RunType type);

// 旋回が通る区画の移り方(経路の計算用)。境目から向きd_inで区画Aに入ったところから、
// 旋回が使う区画を抜けて次の境目に着くまでに区画を移る向き(絶対方位)を順にmovesへ書き、
// その数を返す(最大5。旋回でなければ0)。最後の向きが旋回の後の向き。
// 例: LARGE90_R、d_in=北 → 北, 東, 東(A → 北の区画B → 東の区画C → Cの東の境目)。
// 大回りの指令そのものはCの中心で終わり、最後の1つ(Cの中心 → 境目)は次の直進に入る。
uint8_t RunType_TurnMoves(RunType type, Direction d_in, Direction *moves);
#define RUN_TURN_MOVES_MAX 5

const char *RunType_Name(RunType type);

void RunList_Clear(RunList *list);

// 末尾に追加する。直進が続くときは1つにまとめる。満杯ならfalse。
bool RunList_Push(RunList *list, RunCommand cmd);

// 距離dist[mm]を、速度v_inで入ってv_outで出る直進の時間[s]を、台形加速で求めてtimeへ書く。
// 加速度が足りずにv_in → v_outへ変えられないならfalse(timeは変えない)。
bool RunProfile_StraightTime(const RunProfile *prof, float dist, float v_in, float v_out,
                             float *time);

// 旋回にはさまれた直進の時間[s]。隣の旋回のオフセットを含めた、長さ dist + off_in + off_out の
// 直進として求める。off_in は前の旋回の後オフセット、off_out は次の旋回の前オフセット
// (隣が旋回でなければ0)。加速度が足りなければfalse(timeは「必要な加速度で一様に変える」とした値)。
bool RunProfile_LinkedStraightTime(const RunProfile *prof, float dist,
                                   float v_in, float off_in, float v_out, float off_out,
                                   float *time);

// 指令の列の走行時間[s]を見積もる。スタートとゴールは速度0。
// 直進の入りと出の速度は、前後の旋回の速度(列の端なら0)。速度は直進の間だけで変える。
// times が NULL でなければ、指令ごとの時間を times[i] に書く(STOPは0)。
// 次のどちらかがあればfalseを返し、最初のものの指令の番号を bad_index に書く(NULL可)。
//   - 加速度が足りない直進(その直進の番号)
//   - 旋回どうしが直接つながり、オフセットの中で速度を変えきれない(後の旋回の番号)
// その場合も「必要な加速度で一様に変える」として時間を求めて total に書く。
bool RunList_EstimateTime(const RunList *list, const RunProfile *prof,
                          float *total, float *times, uint16_t *bad_index);

// 区画の経路(Dijkstra_BuildRoute の結果。スタートの中心から、スタートの向きで出発)を
// 最短走行の指令へ置き換えてoutへ書く。最後は必ずRUN_STOP(ゴールの区画の中心で止まる)。
// use_largeがtrueなら、前から順に LARGE180 → LARGE90 の形を探して置き換える。
// 置き換えた大回りの前後で加速度が足りなくなる(スタート直後・ゴール直前、
// 速度の違う旋回と直接つながる所など)ときは、その大回りを小回りに戻す。
// 小回りでも足りなければ、そのまま返す(RunList_EstimateTime で分かる)。
// 最初の指令が直進でない・180°の向き変えがある・長すぎる場合はfalse。
bool RunPath_FromRoute(const CommandList *route, const RunProfile *prof, bool use_large,
                       RunList *out);

// RunPath_FromRoute の逆。最短走行の指令の列を区画の経路(探索の指令の列)に戻す。
// 最初の指令はスタートの中心から始まるものとする。壁にぶつからないかを確かめるのに使う。
// 戻せない(半区画の数が合わない・長すぎる)ならfalse。
bool RunList_ToRoute(const RunList *list, CommandList *out);

// 指令の列を1行1指令でprintfする(デバッグ用)。
void RunList_Print(const RunList *list);

#endif
