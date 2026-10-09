#include "logic/maze/run_path.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include "logic/control/slalom.h"

#define RUN_PI 3.14159265f

RunProfile RunProfile_Default(void) {
    RunProfile p = { 0 };
    p.accel = RUN_ACCEL_MM_S2;
    p.decel = RUN_ACCEL_MM_S2;
    p.vmax  = RUN_VMAX_MM_S;
    p.v_turn[RUN_SMALL90_R]  = RUN_V_SMALL90_MM_S;
    p.v_turn[RUN_SMALL90_L]  = RUN_V_SMALL90_MM_S;
    p.v_turn[RUN_LARGE90_R]  = RUN_V_LARGE90_MM_S;
    p.v_turn[RUN_LARGE90_L]  = RUN_V_LARGE90_MM_S;
    p.v_turn[RUN_LARGE180_R] = RUN_V_LARGE180_MM_S;
    p.v_turn[RUN_LARGE180_L] = RUN_V_LARGE180_MM_S;
    for (int t = RUN_SMALL90_R; t < RUN_TYPE_COUNT; t++) {
        p.turn_len[t]  = RunType_TurnLength((RunType)t);
        p.turn_pre[t]  = 0.0f;
        p.turn_post[t] = 0.0f;
    }
    return p;
}

static void SetTurn(RunTurnSpec turns[RUN_TYPE_COUNT], RunType r, RunType l, const SlalomParams *p,
                    float pre, float post) {
    RunTurnSpec t = { p->v_mm_s, p->omega_dps, p->alpha_dps2, p->angle_deg, pre, post };
    turns[r] = t;
    turns[l] = t;
}

RunProfile RunProfile_ForSpeeds(float vmax, float accel, float small_v, RunTurnSpec turns[RUN_TYPE_COUNT]) {
    RunTurnSpec local[RUN_TYPE_COUNT];
    if (turns == NULL) turns = local;
    for (int t = 0; t < RUN_TYPE_COUNT; t++) {
        RunTurnSpec zero = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        turns[t] = zero;
    }
    float pre, post;

    // 小回り: 探索と同じ形(SLALOM_*)を small_v にする。前後のオフセットはずれのモデルから
    SlalomParams small = { SLALOM_V_MM_S, SLALOM_OMEGA_DPS, SLALOM_ALPHA_DPS2, 90.0f };
    Slalom_ScaleToSpeed(&small, small_v);
    SlalomOffsets so = Slalom_SmallTurnOffsets(&small);
    SetTurn(turns, RUN_SMALL90_R, RUN_SMALL90_L, &small, so.pre_mm, so.post_mm);

    SlalomParams l90 = { FAST_LARGE90_V_MM_S, FAST_LARGE90_OMEGA_DPS, FAST_LARGE90_ALPHA_DPS2, 90.0f };
    SlalomShape sh = Slalom_ComputeShape(&l90);
    Slalom_Turn90Offsets(&sh, SECTION_MM, &pre, &post);
    SetTurn(turns, RUN_LARGE90_R, RUN_LARGE90_L, &l90, pre + FAST_LARGE90_PRE_ADJ_MM, post + FAST_LARGE90_POST_ADJ_MM);

    SlalomParams l180 = { FAST_LARGE180_V_MM_S, 0.0f, FAST_LARGE180_ALPHA_DPS2, 180.0f };
    Slalom_SolveOmegaForSide(&l180, SECTION_MM); // 横にちょうど1区画移る角速度
    sh = Slalom_ComputeShape(&l180);
    Slalom_Turn180Offsets(&sh, SECTION_MM, &pre, &post);
    SetTurn(turns, RUN_LARGE180_R, RUN_LARGE180_L, &l180, pre + FAST_LARGE180_PRE_ADJ_MM, post + FAST_LARGE180_POST_ADJ_MM);

    RunProfile prof = RunProfile_Default();
    prof.accel = accel;
    prof.decel = (accel < FAST_DECEL_MAX_MM_S2) ? accel : FAST_DECEL_MAX_MM_S2; // 減速は滑りやすいので上限まで
    prof.vmax = vmax;
    for (int t = RUN_SMALL90_R; t <= RUN_LARGE180_L; t++) {
        prof.v_turn[t] = turns[t].v_mm_s;
        prof.turn_pre[t] = turns[t].pre_mm;
        prof.turn_post[t] = turns[t].post_mm;
    }
    return prof;
}

float RunProfile_CurveTime(const RunProfile *prof, RunType type) {
    if (!RunType_IsTurn(type)) return 0.0f;
    return (prof->turn_len[type] - prof->turn_pre[type] - prof->turn_post[type]) / prof->v_turn[type];
}

bool RunType_IsLarge(RunType type) {
    return (type >= RUN_LARGE90_R) && (type <= RUN_LARGE180_L);
}

bool RunType_IsTurn(RunType type) {
    return (type >= RUN_SMALL90_R) && (type < RUN_TYPE_COUNT);
}

float RunType_TurnLength(RunType type) {
    const float half = SECTION_MM * 0.5f;
    switch (type) {
        case RUN_SMALL90_R:
        case RUN_SMALL90_L:  return RUN_PI * 0.5f * half;                  // 半径1/2区画の90°
        case RUN_LARGE90_R:
        case RUN_LARGE90_L:  return RUN_PI * 0.5f * SECTION_MM;            // 半径1区画の90°
        case RUN_LARGE180_R:
        case RUN_LARGE180_L: return SECTION_MM + RUN_PI * half;             // 半区画 + 柱を回る半円 + 半区画
        default:             return 0.0f;
    }
}

int RunType_QuarterTurns(RunType type) {
    switch (type) {
        case RUN_SMALL90_R:
        case RUN_LARGE90_R:  return 1;
        case RUN_SMALL90_L:
        case RUN_LARGE90_L:  return -1;
        case RUN_LARGE180_R: return 2;
        case RUN_LARGE180_L: return -2;
        default:             return 0;
    }
}

uint8_t RunType_TurnMoves(RunType type, Direction d_in, Direction *moves) {
    int q = RunType_QuarterTurns(type) > 0 ? 1 : -1;
    Direction d1 = Dir_Turn(d_in, q);
    switch (type) {
        case RUN_SMALL90_R:
        case RUN_SMALL90_L:
            moves[0] = d1;
            return 1;
        case RUN_LARGE90_R:
        case RUN_LARGE90_L:
            moves[0] = d_in;
            moves[1] = d1;
            moves[2] = d1;
            return 3;
        case RUN_LARGE180_R:
        case RUN_LARGE180_L: // A → B(前) → C(横) → D(後ろ)
            moves[0] = d_in;
            moves[1] = d1;
            moves[2] = Dir_Opposite(d_in);
            moves[3] = Dir_Opposite(d_in);
            return 4;
        default:
            return 0;
    }
}

const char *RunType_Name(RunType type) {
    switch (type) {
        case RUN_STOP:       return "STOP";
        case RUN_STRAIGHT:   return "STRAIGHT";
        case RUN_SMALL90_R:  return "SMALL90_R";
        case RUN_SMALL90_L:  return "SMALL90_L";
        case RUN_LARGE90_R:  return "LARGE90_R";
        case RUN_LARGE90_L:  return "LARGE90_L";
        case RUN_LARGE180_R: return "LARGE180_R";
        case RUN_LARGE180_L: return "LARGE180_L";
        default:             return "?";
    }
}

void RunList_Clear(RunList *list) {
    list->count = 0;
}

bool RunList_Push(RunList *list, RunCommand cmd) {
    if (cmd.type == RUN_STRAIGHT && list->count > 0) {
        RunCommand *last = &list->items[list->count - 1];
        if (last->type == RUN_STRAIGHT && (uint16_t)last->halves + cmd.halves <= 0xFFu) {
            last->halves = (uint8_t)(last->halves + cmd.halves);
            return true;
        }
    }
    if (list->count >= RUN_LIST_MAX) return false;
    list->items[list->count++] = cmd;
    return true;
}

bool RunProfile_StraightTime(const RunProfile *prof, float dist, float v_in, float v_out,
                             float *time) {
    const float a = prof->accel;
    const float d = prof->decel;
    if (dist <= 0.0f) return false;
    // 速度を変えるのに要る距離が足りなければ無理(上げるなら加速度、下げるなら減速度で)
    if (v_out > v_in && v_out * v_out - v_in * v_in > 2.0f * a * dist) return false;
    if (v_out < v_in && v_in * v_in - v_out * v_out > 2.0f * d * dist) return false;

    // 加速して減速する三角形の頂点の速度(dist = (vp² − v_in²)/2a + (vp² − v_out²)/2d)。vmaxを超えるなら台形にする。
    float v_peak = sqrtf((2.0f * dist + v_in * v_in / a + v_out * v_out / d) / (1.0f / a + 1.0f / d));
    if (v_peak > prof->vmax) v_peak = prof->vmax;
    if (v_peak < v_in)  v_peak = v_in;  // v_in・v_outがvmaxを超えている場合
    if (v_peak < v_out) v_peak = v_out;

    float d_acc    = (v_peak * v_peak - v_in * v_in) / (2.0f * a);
    float d_dec    = (v_peak * v_peak - v_out * v_out) / (2.0f * d);
    float d_cruise = dist - d_acc - d_dec;
    if (d_cruise < 0.0f) d_cruise = 0.0f; // 丸め誤差

    *time = (v_peak - v_in) / a + (v_peak - v_out) / d + d_cruise / v_peak;
    return true;
}

// 指令iの前後で、直進の入りと出の速度に使う値(旋回ならその速度、列の端やSTOPなら0)
static float BoundarySpeed(const RunList *list, const RunProfile *prof, int i) {
    if (i < 0 || i >= (int)list->count) return 0.0f;
    RunType t = (RunType)list->items[i].type;
    return RunType_IsTurn(t) ? prof->v_turn[t] : 0.0f;
}

// 指令iが旋回なら、その前オフセット・後オフセット(旋回でなければ0)
static float TurnPre(const RunList *list, const RunProfile *prof, int i) {
    if (i < 0 || i >= (int)list->count) return 0.0f;
    RunType t = (RunType)list->items[i].type;
    return RunType_IsTurn(t) ? prof->turn_pre[t] : 0.0f;
}

static float TurnPost(const RunList *list, const RunProfile *prof, int i) {
    if (i < 0 || i >= (int)list->count) return 0.0f;
    RunType t = (RunType)list->items[i].type;
    return RunType_IsTurn(t) ? prof->turn_post[t] : 0.0f;
}

bool RunProfile_LinkedStraightTime(const RunProfile *prof, float dist,
                                   float v_in, float off_in, float v_out, float off_out,
                                   float *time) {
    // オフセットを含めた長い直進として加減速する
    float ext = dist + off_in + off_out;
    if (ext <= 0.0f) { // 長さ0(オフセットのない旋回どうしが直接つながる): 速度は変えられない
        *time = 0.0f;
        return v_in == v_out;
    }
    float t;
    bool ok = RunProfile_StraightTime(prof, ext, v_in, v_out, &t);
    if (!ok) t = 2.0f * ext / (v_in + v_out); // 加速度が足りない: 必要な加速度で一様に変える
    *time = t;
    return ok;
}

bool RunList_EstimateTime(const RunList *list, const RunProfile *prof,
                          float *total, float *times, uint16_t *bad_index) {
    bool ok = true;
    float sum = 0.0f;

    for (uint16_t i = 0; i < list->count; i++) {
        const RunCommand *c = &list->items[i];
        RunType t = (RunType)c->type;
        float dt = 0.0f;

        if (t == RUN_STRAIGHT) {
            float dist  = (float)c->halves * SECTION_MM * 0.5f;
            float v_in  = BoundarySpeed(list, prof, (int)i - 1);
            float v_out = BoundarySpeed(list, prof, (int)i + 1);
            float off_in  = (i > 0) ? TurnPost(list, prof, (int)i - 1) : 0.0f;
            float off_out = TurnPre(list, prof, (int)i + 1);
            if (!RunProfile_LinkedStraightTime(prof, dist, v_in, off_in, v_out, off_out, &dt)) {
                if (ok && bad_index != NULL) *bad_index = i;
                ok = false;
            }
        } else if (RunType_IsTurn(t)) {
            dt = RunProfile_CurveTime(prof, t);
            // 旋回どうしが直接つながるときは、前の旋回の後オフセット + この旋回の前オフセットを
            // 長さ0の直進として加減速する(速度が違えば、そこで変える)
            if (i > 0 && RunType_IsTurn((RunType)list->items[i - 1].type)) {
                float link;
                if (!RunProfile_LinkedStraightTime(prof, 0.0f, BoundarySpeed(list, prof, (int)i - 1),
                                                   TurnPost(list, prof, (int)i - 1),
                                                   prof->v_turn[t], prof->turn_pre[t], &link)) {
                    if (ok && bad_index != NULL) *bad_index = i;
                    ok = false;
                }
                dt += link;
            }
        }

        if (times != NULL) times[i] = dt;
        sum += dt;
    }

    *total = sum;
    return ok;
}

// ---- 区画の経路 → 最短走行の指令 ----

// 大回りを使ってよい範囲(区画ごと)。加速度が足りないときに1段ずつ下げる。
enum { LARGE_ALL = 0, LARGE_NO180, LARGE_NONE };

// 作業領域(約1.3KB)。Dijkstraの作業領域と同じく、同時に呼んではいけない。
static int8_t   s_turn[RUN_LIST_MAX];     // 区画kでの曲がり方(0、右+1、左-1)。kは経路の何区画目か
static uint8_t  s_limit[RUN_LIST_MAX];    // 区画kから始まる大回りの制限(LARGE_*)
static uint16_t s_item_cell[RUN_LIST_MAX]; // 指令の番号 → その旋回が始まる区画(大回りでなければ0)

// 区画k, k+1, ... の曲がり方が pattern(sは右+1か左-1)と同じならtrue
static bool MatchTurns(uint16_t k, uint16_t last, const int8_t *pattern, uint16_t len, int8_t s) {
    if (k + len - 1 > last) return false;
    for (uint16_t i = 0; i < len; i++) {
        if (s_turn[k + i] != pattern[i] * s) return false;
    }
    return true;
}

static bool PushCmd(RunList *out, RunType type, uint8_t halves, uint16_t large_cell) {
    RunCommand c = { (uint8_t)type, halves };
    if (!RunList_Push(out, c)) return false;
    s_item_cell[out->count - 1] = large_cell;
    return true;
}

// 区画1〜last(lastの次がゴールの区画)を、s_limitに従って指令にする
static bool BuildRun(uint16_t last, bool use_large, RunList *out) {
    static const int8_t kPat180[4] = { 0, 1, 1, 0 };
    static const int8_t kPat90[3]  = { 0, 1, 0 };

    RunList_Clear(out);
    if (!PushCmd(out, RUN_STRAIGHT, 1, 0)) return false; // スタートの中心 → 境目

    // at_center: 前の大回りが区画kの中心で終わっていて、そこにいる
    // (区画kの前半分は使い済み。次の大回りはここから直接始められる)
    bool at_center = false;
    uint16_t k = 1;
    while (k <= last) {
        bool done = false;
        if (use_large && k + 1 <= last) {
            int8_t s = s_turn[k + 1]; // 大回りなら2区画目で曲がる
            if (s != 0) {
                bool right = (s > 0);
                // 大回りは区画kの中心から最後の区画の中心まで。前後の半区画は直進に入れる
                RunType large = RUN_STOP;
                uint16_t used = 0;
                if (s_limit[k] == LARGE_ALL && MatchTurns(k, last, kPat180, 4, s)) {
                    large = right ? RUN_LARGE180_R : RUN_LARGE180_L;
                    used = 4;
                } else if (s_limit[k] <= LARGE_NO180 && MatchTurns(k, last, kPat90, 3, s)) {
                    large = right ? RUN_LARGE90_R : RUN_LARGE90_L;
                    used = 3;
                }
                if (used > 0) {
                    if (!at_center && !PushCmd(out, RUN_STRAIGHT, 1, 0)) return false; // 境目 → 中心
                    if (!PushCmd(out, large, 0, k)) return false;
                    // 最後の区画の中心にいる。その区画から次の大回りを始められるので、kはそこへ
                    k = (uint16_t)(k + used - 1);
                    at_center = true;
                    done = true;
                }
            }
        }
        if (!done && at_center) {
            // 大回りの後の区画の残り半分(中心 → 境目)
            if (!PushCmd(out, RUN_STRAIGHT, 1, 0)) return false;
            k++;
            at_center = false;
            done = true;
        }
        if (!done) {
            bool ok;
            if (s_turn[k] == 0)     ok = PushCmd(out, RUN_STRAIGHT, 2, 0);
            else if (s_turn[k] > 0) ok = PushCmd(out, RUN_SMALL90_R, 0, 0);
            else                    ok = PushCmd(out, RUN_SMALL90_L, 0, 0);
            if (!ok) return false;
            k++;
        }
    }

    // 最後の境目 → ゴールの区画の中心で止まる
    if (!PushCmd(out, RUN_STRAIGHT, 1, 0)) return false;
    return PushCmd(out, RUN_STOP, 0, 0);
}

// 指令iが大回りなら、始まる区画の制限を1段下げる。下げたらtrue。
static bool Demote(const RunList *list, int i) {
    if (i < 0 || i >= (int)list->count) return false;
    RunType t = (RunType)list->items[i].type;
    uint16_t cell = s_item_cell[i];
    if (t == RUN_LARGE180_R || t == RUN_LARGE180_L) {
        s_limit[cell] = LARGE_NO180;
        return true;
    }
    if (t == RUN_LARGE90_R || t == RUN_LARGE90_L) {
        s_limit[cell] = LARGE_NONE;
        return true;
    }
    return false;
}

bool RunPath_FromRoute(const CommandList *route, const RunProfile *prof, bool use_large,
                       RunList *out) {
    // 1区画ずつの曲がり方に展開する。moves[0]はスタートから出るときの向き変え。
    uint16_t moves = 0;
    for (uint16_t i = 0; i < route->count; i++) {
        const Action *a = &route->items[i];
        if (a->type == ACTION_STOP) break;
        int q = Action_QuarterTurns((ActionType)a->type);
        if (q == 2) return false; // 180°の向き変えは最短走行では扱わない
        uint16_t n = (a->type == ACTION_FORWARD) ? a->cells : 1;
        for (uint16_t j = 0; j < n; j++) {
            if (moves + 1u >= RUN_LIST_MAX) return false; // s_limit[moves] まで使う
            s_turn[moves++] = (int8_t)((j == 0) ? q : 0);
        }
    }

    RunList_Clear(out);
    if (moves == 0) { // スタートがゴール
        RunCommand stop = { RUN_STOP, 0 };
        return RunList_Push(out, stop);
    }
    if (s_turn[0] != 0) return false; // スタートの中心では曲がれない(その場旋回になる)

    // s_turn[k] (k = 1 .. moves-1) が区画kでの曲がり方。区画movesがゴール。
    uint16_t last = (uint16_t)(moves - 1);
    for (uint16_t k = 0; k <= moves; k++) s_limit[k] = LARGE_ALL;

    // 加速度が足りない直進があれば、隣の大回りを下げて作り直す
    // (1回で1つの区画の制限が1段下がるので、2 × 区画数 回で必ず終わる)
    for (uint16_t guard = 0; guard <= 2u * moves; guard++) {
        if (!BuildRun(last, use_large, out)) return false;
        float total;
        uint16_t bad;
        if (RunList_EstimateTime(out, prof, &total, NULL, &bad)) return true;
        // 直進なら前後の旋回、旋回(速度の違う旋回が直接つながる)ならその旋回と1つ前
        bool demoted = Demote(out, (int)bad - 1);
        if (out->items[bad].type == RUN_STRAIGHT) demoted = Demote(out, (int)bad + 1) || demoted;
        else                                      demoted = Demote(out, (int)bad) || demoted;
        if (!demoted) return true; // 小回りでも足りない。そのまま返す
    }
    return true;
}

bool RunList_ToRoute(const RunList *list, CommandList *out) {
    CommandList_Clear(out);
    bool at_center = true; // スタートの中心から
    Direction heading = DIR_NORTH; // 相対の向きしか使わないので、どの向きから始めてもよい

    for (uint16_t i = 0; i < list->count; i++) {
        const RunCommand *c = &list->items[i];
        RunType t = (RunType)c->type;
        if (t == RUN_STOP) break;

        if (t == RUN_STRAIGHT) {
            // 中心 → 境目 の半区画で、次の区画へ1つ移る
            for (uint8_t h = 0; h < c->halves; h++) {
                if (at_center && !CommandList_Push(out, Action_Move(0), true)) return false;
                at_center = !at_center;
            }
        } else if (RunType_IsTurn(t)) {
            // 小回りは境目から境目、大回りは中心から中心
            bool large = RunType_IsLarge(t);
            if (at_center != large) return false;
            Direction moves[RUN_TURN_MOVES_MAX];
            uint8_t n = RunType_TurnMoves(t, heading, moves);
            if (large) n--; // 最後の区画の中心で終わる(その先の境目へは次の直進で移る)
            for (uint8_t k = 0; k < n; k++) {
                if (!CommandList_Push(out, Action_Move((int)moves[k] - (int)heading), true)) return false;
                heading = moves[k];
            }
        }
    }
    if (!at_center) return false; // 区画の中心で止まっていない

    Action stop = { ACTION_STOP, 0 };
    return CommandList_Push(out, stop, false);
}

void RunList_Print(const RunList *list) {
    for (uint16_t i = 0; i < list->count; i++) {
        const RunCommand *c = &list->items[i];
        if (c->type == RUN_STRAIGHT) {
            printf("%3u: %s x%u/2\r\n", (unsigned)i, RunType_Name((RunType)c->type),
                   (unsigned)c->halves);
        } else {
            printf("%3u: %s\r\n", (unsigned)i, RunType_Name((RunType)c->type));
        }
    }
}

bool RunPath_FromKnownRun(const Direction *moves, uint16_t m, const RunProfile *prof, bool use_large, RunList *out) {
    static CommandList route;
    CommandList_Clear(&route);
    bool ok = CommandList_Push(&route, Action_Move(0), true) && // P → C0
              CommandList_Push(&route, Action_Move(0), true);   // C0 → C1
    for (uint16_t k = 1; k <= m && ok; k++) {
        ok = CommandList_Push(&route, Action_Move((int)moves[k] - (int)moves[k - 1u]), true);
    }
    Action stop = { ACTION_STOP, 0 };
    if (!ok || !CommandList_Push(&route, stop, false)) return false;
    if (!RunPath_FromRoute(&route, prof, use_large, out)) return false;
    int last = -1;
    for (int i = 0; i < (int)out->count; i++) {
        if (out->items[i].type == RUN_STOP) break;
        last = i;
    }
    if (last < 0 || out->items[last].type != RUN_STRAIGHT || out->items[last].halves < 2u) return false;
    out->items[last].halves--;
    return true;
}
