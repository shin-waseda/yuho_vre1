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
//
// ---- 斜め(2026-11-07)----
// 斜めの経路は、区画の境目の真ん中だけを通る(区画の中心も柱も通らない)。斜めの位置(斜めの点)は
// 「区画に向き d で入る境目の真ん中を、d から左右どちらかへ 45° 傾いた向きで通る所」。
// 斜めの直進 DIAG は斜めの点から斜めの点までを halves 個(1個 = 1区画/√2 ≈ 127mm。区画を1つ斜めに横切る)。
// 斜めの区間では、区画ごとの曲がり方が R L R L …(ジグザグ)になる。1区画が DIAG の1個。
// 斜めへの入りと出は境目(小回りと同じ側)から始まり・境目で終わる。V90 は斜めの点から斜めの点。
//
// | 指令      | 区画ごとの曲がり方 | 動き(右の場合。前 = 曲がり始めの向き、横 = 曲がる側)           | 角までの折れ線   |
// |-----------|--------------------|----------------------------------------------------------------|------------------|
// | D45IN     | 0 R  (→ L …)      | 境目 → 前 1.5区画・横 0.5区画の斜めの点                       | 1区画 + 1/√2区画 |
// | D135IN    | R R  (→ L …)      | 境目 → 前 0・横 1区画の斜めの点(角は同じ区画の向こうの境目)   | 1区画 + √2区画   |
// | D45OUT    | R 0                | 斜めの点 → 境目(角は R の区画の出口の境目)                    | 1/√2区画 + 1区画 |
// | D135OUT   | R R                | 斜めの点 → 境目                                               | √2区画 + 1区画   |
// | DV90      | (R) 0 (R)          | 斜めの点 → 斜めの点(角は 0 の区画の、出ない側の境目)          | 1/√2 + 1/√2区画  |
// | DV90P     | (L) R R (L)        | 斜めの点 → 斜めの点(角は R R の間の境目。内側に柱が1本入る)    | 1/√2 + 1/√2区画  |
//
// 入りの後と V90 の後は、斜めの点から「次の区画をどちらへ曲がって抜けるか」が決まっている(ジグザグの続き)。
// 出(D45OUT・D135OUT)は、その続きの向きへ曲がる区画から始まる(上の表の R)。V90 は 0 の区画で、前後の区画は
// V90 と同じ向きに曲がる(L R 0 R L のように、ジグザグの続きと逆の向きに 90° 曲がる)。
// 同じ向きが2つ続く所(L R R L)は柱を回る V90(DV90P)。角は R R の間で通り抜ける境目で、内側に柱が1本入るので、
// DV90 より小さく回る(柱との隙間 ≒ 半区画 − 0.41 × 回る半径。2026-11-07 に使うようにした。前は D135OUT で斜めを抜けていた)。
// R R の後が斜めの続き(L …)でなければ、D135OUT で斜めを抜ける。
typedef enum {
    RUN_STOP = 0,     // 止まる(経路の終わり)
    RUN_STRAIGHT,     // halves × 半区画 まっすぐ進む
    RUN_SMALL90_R,
    RUN_SMALL90_L,
    RUN_LARGE90_R,
    RUN_LARGE90_L,
    RUN_LARGE180_R,
    RUN_LARGE180_L,
    // 斜め(番号は上のものを変えないように後ろへ足す。maze_sim の画面は上の番号を使う)
    RUN_DIAG,         // halves × 斜めの1区間(1区画/√2) 斜めにまっすぐ進む
    RUN_D45IN_R,
    RUN_D45IN_L,
    RUN_D45OUT_R,
    RUN_D45OUT_L,
    RUN_D135IN_R,
    RUN_D135IN_L,
    RUN_D135OUT_R,
    RUN_D135OUT_L,
    RUN_DV90_R,
    RUN_DV90_L,
    // 超信地旋回(2026-11-07): 区画の中心で止まって、その場で 90° 回る(中心から中心。区画は移らない)。
    // 速さは 0 で、曲がる時間は数えない(RunProfile の v_turn は 0。maze_sim は超信地旋回の時間を数えない決まり)
    RUN_PIVOT90_R,
    RUN_PIVOT90_L,
    // 柱を回る V90(2026-11-07。上の表の DV90P)
    RUN_DV90P_R,
    RUN_DV90P_L,
    RUN_TYPE_COUNT,
} RunType;

// 2バイト。typeはRunTypeの値。
typedef struct {
    uint8_t type;   // RunType
    uint8_t halves; // 直進の長さ(半区画単位、1以上。DIAG は斜めの区間の数)。旋回とSTOPは0
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
// 時間の数え方(実機の走り方 = app/search_run の RunList_Drive に合わせる。2026-10-10 に最短走行のログで確かめた):
//   - 旋回の時間は、曲線の部分(turn_len − 前後オフセット)を旋回の速度で走る時間
//     (RunProfile_CurveTime)。
//   - 直進の時間は、隣の旋回のオフセットを旋回の速度のまま走る時間 + 間の直進を台形加速で走る時間
//     (RunProfile_LinkedStraightTime)。オフセットは直進の側に数える。
//   - 旋回どうしが直接つながるときは、前の旋回の後オフセット + 次の旋回の前オフセットを
//     長さ0の直進とみなし、その中で速度を変える(速度が同じなら、旋回の速度のまま走る)。
//   オフセットを旋回の側に数えて直進から引く分け方だと、直結の区間で加速した分が
//   負の時間になり、Dijkstra で扱えない。この分け方なら、どの部分の時間も0以上になる。
// 既定値は円弧だけの理想的な形(長さは RunType_TurnLength()、オフセットは0)。斜めの旋回は、角までの折れ線の短い方に
// 収まる一番大きな円弧にし、長い方の残りを前か後ろのオフセットにする(RunType_TurnLength() はオフセットを含む長さ)。
// RunProfile_ForSpeeds は実機の旋回の形を使う(turn_len = 前オフセット + 曲がる道のり(速さ × 曲がる時間) + 後オフセット)。
// 斜めの直進(DIAG)も、最高速度・加速度・減速度は直進と同じ(2026-11-07 のユーザーの決め)。
typedef struct {
    float accel;                     // [mm/s²] 直進の加速度
    float decel;                     // [mm/s²] 直進の減速度(RunProfile_Default は accel と同じ)
    float vmax;                      // [mm/s] 直進の最高速度
    float v_turn[RUN_TYPE_COUNT];    // [mm/s] 旋回の種類ごとの速度
    float turn_len[RUN_TYPE_COUNT];  // [mm] 旋回の経路の長さ(オフセットを含む)
    float turn_pre[RUN_TYPE_COUNT];  // [mm] 前オフセット
    float turn_post[RUN_TYPE_COUNT]; // [mm] 後オフセット
} RunProfile;

RunProfile RunProfile_Default(void);

// 旋回の動き(機体がスラロームとして走る値)。最短走行・探索の既知の区間(app/search_run)と maze_sim で同じものを使う。
typedef struct {
    float v_mm_s;     // 並進の速さ
    float omega_dps;  // 最高角速度
    float alpha_dps2; // 角加速度
    float angle_deg;  // 45・90・135・180
    float pre_mm;     // 前のオフセット
    float post_mm;    // 後ろのオフセット
} RunTurnSpec;

// 直進の最高速度 vmax・加速度 accel・小回りの速さ small_v で走るときの、旋回の動き(turns[RunType]、右と左に同じ値)と、
// それを使う RunProfile を作る。小回りは探索と同じ形(SLALOM_*)を small_v にして、前後のオフセットはスラロームのずれの
// モデル(Slalom_SmallTurnOffsets)で計算する。大回り 90°・180° は FAST_LARGE* の速さと形(+ *_ADJ)。
// 2026-11-08: 大回りの前後のオフセットにもスリップのモデルを入れた(Slalom_Large90Offsets / Slalom_Large180Offsets)。
// 斜めの旋回は FAST_D45*・FAST_D135*・FAST_DV90* の速さと形で、前後のオフセットは角までの折れ線から(Slalom_TurnOffsets。
// 入りと出は同じ形で、折れ線の前後が入れ替わる)。斜めの直進の最高速度は直進と同じ vmax。
// turns は NULL でもよい。
RunProfile RunProfile_ForSpeeds(float vmax, float accel, float small_v, RunTurnSpec turns[RUN_TYPE_COUNT]);

// RunProfile_ForSpeeds と同じだが、大回り 90°・180° の速さも選ぶ(形は FAST_LARGE* を曲がる軌跡を変えずにその速さにしたもの)
RunProfile RunProfile_ForSpeedsLarge(float vmax, float accel, float small_v, float large90_v, float large180_v,
                                     RunTurnSpec turns[RUN_TYPE_COUNT]);

// 探索の既知の区間(SearchPlanner_KnownRun の moves[0..m])を、最短走行の指令の列にする。区画 C0 の入口の境界にいて
// C0 をまっすぐ抜ける所から始まるので、1つ手前の区画 P の真ん中から「P → C0 → C1 → moves[1..m]」の経路として作り、
// 最後の直進を半区画短くして、終わりの区画の真ん中ではなく入口の境界で終わるようにする(先頭の半区画 P → C0 は走り済み)。
// 作れなければ false(最後が直進でない、直進が短すぎるなど)。
bool RunPath_FromKnownRun(const Direction *moves, uint16_t m, const RunProfile *prof, bool use_large, RunList *out);

// 旋回ならtrue(斜めの旋回を含む。直進・斜めの直進・STOPはfalse)
bool RunType_IsTurn(RunType type);

// 旋回の経路の長さ[mm](理想的な円弧。小回りは境目から境目、大回りは中心から中心、斜めは表の位置どうし)。
// 旋回でなければ0。
float RunType_TurnLength(RunType type);

// 大回り(中心から中心へ曲がる旋回)ならtrue
bool RunType_IsLarge(RunType type);

// 斜めの点から始まる指令(DIAG・D45OUT・D135OUT・DV90・DV90P)ならtrue
bool RunType_StartsDiag(RunType type);
// 斜めの点で終わる指令(DIAG・D45IN・D135IN・DV90・DV90P)ならtrue
bool RunType_EndsDiag(RunType type);

// 旋回の曲線の部分(オフセットを除く)を旋回の速度で走る時間[s]。旋回でなければ0(速さが0の超信地旋回も0)。
float RunProfile_CurveTime(const RunProfile *prof, RunType type);

// 旋回の向きの変化(90°単位、時計回りが正)。直進・STOP・斜めの旋回は0(斜めは RunType_TurnDeg)。
int RunType_QuarterTurns(RunType type);

// 旋回の向きの変化[deg](時計回りが正)。直進・STOPは0。
float RunType_TurnDeg(RunType type);

// 斜めの旋回の、曲がり始めの所から角(折れ線の折れる点)まで・角から曲がり終わりの所までの長さ[mm](表の右の欄)。
// 斜めの旋回でなければ false。
bool RunType_DiagLegs(RunType type, float *leg_in_mm, float *leg_out_mm);

// 斜めの旋回 type(右・左どちらでも)の動きを、FAST_D45*・FAST_D135*・FAST_DV90*・FAST_DV90P* の形を曲がる軌跡を変えずに
// 速さ v_mm_s にして *out に書く(角速度は速さに比例、角加速度は2乗に比例。v_mm_s が 0 以下なら FAST_D* の速さのまま)。
// 前後のオフセットは角までの折れ線から(RunProfile_ForSpeeds と同じ。軌跡が変わらないので速さによらない)。
// 斜めの旋回でなければ false。SLIP の CAMERA の DIAG(2026-11-22)で、旋回の速さを変えて走るのに使う
bool RunType_DiagTurnSpec(RunType type, float v_mm_s, RunTurnSpec *out);

// 旋回が通る区画の移り方(経路の計算用)。境目から向きd_inで区画Aに入ったところから、
// 旋回が使う区画を抜けて次の境目に着くまでに区画を移る向き(絶対方位)を順にmovesへ書き、
// その数を返す(最大5。旋回でなければ0)。最後の向きが旋回の後の向き。
// 例: LARGE90_R、d_in=北 → 北, 東, 東(A → 北の区画B → 東の区画C → Cの東の境目)。
// 大回りの指令そのものはCの中心で終わり、最後の1つ(Cの中心 → 境目)は次の直進に入る。
// 斜めの点から始まる旋回(D45OUT・D135OUT・DV90)の d_in は、その斜めの点で入る区画の向き。
// 斜めの点で終わる旋回(D45IN・D135IN・DV90)は、最後の向きが終わりの斜めの点で入る区画の向き。
// 例: D45IN_R、d_in=北 → 北, 東(A → 北の区画B → Bの東の境目。そこを北東へ通る)。
uint8_t RunType_TurnMoves(RunType type, Direction d_in, Direction *moves);
#define RUN_TURN_MOVES_MAX 5

// 斜めの点(区画に向き d_in で入る境目)から、ジグザグの続きで次の区画を抜ける向きの回り方(右 +1、左 −1)。
// 斜めの点で終わる指令 type(D45IN・D135IN・DV90)の後なら、その指令で決まる。斜めの直進 DIAG を n 区間進むと、
// 1区間ごとに符号が入れ替わり、区画を抜ける向きは Dir_Turn(d_in, z) と d_in を交互にくり返す。
int RunType_DiagNextTurn(RunType type);

const char *RunType_Name(RunType type);

void RunList_Clear(RunList *list);

// 末尾に追加する。直進が続くときは1つにまとめる。満杯ならfalse。
bool RunList_Push(RunList *list, RunCommand cmd);

// 距離dist[mm]を、速度v_inで入ってv_outで出る直進の時間[s]を、台形加速で求めてtimeへ書く。
// 加速度が足りずにv_in → v_outへ変えられないならfalse(timeは変えない)。
bool RunProfile_StraightTime(const RunProfile *prof, float dist, float v_in, float v_out,
                             float *time);

// 旋回にはさまれた直進の時間[s]。off_in は前の旋回の後オフセット、off_out は次の旋回の前オフセット
// (隣が旋回でなければ0)。時間は、オフセットを旋回の速度(v_in / v_out)のまま走り、長さ dist の直進で
// 加減速するとして求める(dist が0なら、オフセットの中で速度を変える)。
// 速度を変えられるか(戻り値)は、長さ dist + off_in + off_out の直進で決める。
// 加速度が足りなければfalse(timeは「必要な加速度で一様に変える」とした値)。
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

// RunList_EstimateTime の、列の始めと終わりの速度を v_start / v_end にしたもの(探索の既知の区間のように、
// 走りながら始まって走りながら終わる列。maze_sim が使う)。RunList_EstimateTime は v_start = v_end = 0
bool RunList_EstimateTimeEnds(const RunList *list, const RunProfile *prof, float v_start, float v_end,
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

// RunPath_FromRoute に斜めを足したもの(use_diag が false なら RunPath_FromRoute と同じ)。
// 境目にいる所ごとに、前から順に 斜め → 大回り → 小回り の形を探して置き換える。斜めは、入り(D45IN・D135IN)の後の
// 区画の曲がり方を斜めのまま最後までたどれて、出(D45OUT・D135OUT)で境目に戻れるときだけ使う(ゴールの区画には斜めで入らない)。
// 斜めの区間の中では、斜めのまま進める限り進む(ジグザグの続き → V90、どちらもだめなら出)。
// 加速度が足りないときは、大回りと同じように、足りない所の斜めの区間を1つずつやめて(その入りの区画では斜めを使わない)作り直す。
// その後、斜めの区間ごとに、やめた場合(大回り・小回りになる)の列も作って見積もりの時間を比べ、速くなるならやめる
// (変わらなくなるまでくり返す。同じ時間なら斜めのまま)。作業領域(約1.6KB)は run_path.c に1つだけ置く。
bool RunPath_FromRouteEx(const CommandList *route, const RunProfile *prof, bool use_large, bool use_diag,
                         RunList *out);

// 区画の経路を、曲がる区画の中心で止まって超信地旋回する指令の列にする(直進は中心から中心、STRAIGHT と PIVOT90 だけ)。
// 最初の指令が直進でない・180°の向き変えがある・長すぎる場合はfalse。
bool RunPath_FromRoutePivot(const CommandList *route, RunList *out);

// 超信地旋回(PIVOT90)ならtrue
bool RunType_IsPivot(RunType type);

// RunPath_FromRoute の逆。最短走行の指令の列を区画の経路(探索の指令の列)に戻す。
// 最初の指令はスタートの中心から始まるものとする。壁にぶつからないかを確かめるのに使う(斜めの指令も戻せる)。
// 戻せない(半区画の数が合わない・長すぎる)ならfalse。
bool RunList_ToRoute(const RunList *list, CommandList *out);

// 指令の列を1行1指令でprintfする(デバッグ用)。
void RunList_Print(const RunList *list);

#endif
