#include "search_time.h"

#include <math.h>
#include <stddef.h>

#include "params.h"
#include "logic/control/slalom.h"

#define HALF_SECTION_MM (SECTION_MM * 0.5f)

SearchTimeParams SearchTime_DefaultParams(void) {
    SearchTimeParams p = { SEARCH_V_MM_S, SLALOM_V_MM_S, SEARCH_ACCEL_MM_S2, true, SEARCH_TIME_ROUND };
    return p;
}

// ---- 動きの時間 ----

// 台形の速度で dist 進む時間(v_in で入って v_out で出る)。加速度が足りなければ、必要な加速度で一様に変える
static float StraightTime(float dist, float v_in, float v_out, float v_max, float accel, float decel) {
    if (dist <= 0.0f) return 0.0f;
    RunProfile p = { 0 };
    p.accel = accel;
    p.decel = decel;
    p.vmax = v_max;
    float t;
    if (!RunProfile_StraightTime(&p, dist, v_in, v_out, &t)) t = 2.0f * dist / (v_in + v_out);
    return t;
}

// 探索の直進(機体の StartStraightTo: 最高速度は直進の速さと終わりの速さの速い方、加速度と減速度は同じ)
static float SearchStraight(const SearchTime *st, float dist, float v_in, float v_out) {
    float v_max = (v_out > st->p.v_mm_s) ? v_out : st->p.v_mm_s;
    return StraightTime(dist, v_in, v_out, v_max, st->p.accel_mm_s2, st->p.accel_mm_s2);
}

// 止まった状態から止まるまで、台形の角速度で回る時間
static float RotateTime(float angle_deg, float omega_dps, float alpha_dps2) {
    float a = fabsf(angle_deg);
    if (a * alpha_dps2 >= omega_dps * omega_dps) return a / omega_dps + omega_dps / alpha_dps2;
    return 2.0f * sqrtf(a / alpha_dps2); // 最高角速度に届かない(三角形)
}

// 超信地旋回(機体の Pivot: 待ち → 回る → 待ち)
static void AddPivot(SearchTime *st, float angle_deg) {
    float t = 2.0f * (float)SEARCH_TURN_WAIT_MS * 1e-3f + RotateTime(angle_deg, SEARCH_TURN_OMEGA_DPS, SEARCH_TURN_ALPHA_DPS2);
    st->t_s += t;
    st->t_pivot += t;
}

// 尻当て(機体の SetPosition: 決めた時間下がって押し当て → 待ち → 真ん中まで進んで止まる)
static void AddSetPos(SearchTime *st) {
    float t = (float)(SEARCH_SETPOS_BACK_MS + SEARCH_SETPOS_SETTLE_MS) * 1e-3f
            + SearchStraight(st, SEARCH_SETPOS_FRONT_MM, 0.0f, 0.0f);
    st->t_s += t;
    st->t_setpos += t;
    st->n_setpos++;
}

static void AddStraight(SearchTime *st, float t) {
    st->t_s += t;
    st->t_straight += t;
}

// 真ん中で止まる(機体の StopAtCenter。境界からスラロームの速さで半区画減速する)
static void StopAtCenter(SearchTime *st) {
    if (st->at_center) return;
    AddStraight(st, SearchStraight(st, HALF_SECTION_MM, st->p.turn_v_mm_s, 0.0f));
    st->at_center = true;
}

// 次の境界へ(機体の GoToNextBoundary)。境界からなら cells 区画、真ん中からなら半区画 + (cells − 1) 区画
static void GoToNextBoundary(SearchTime *st, uint8_t cells) {
    float vt = st->p.turn_v_mm_s;
    if (st->at_center) {
        AddStraight(st, SearchStraight(st, HALF_SECTION_MM + SECTION_MM * (float)(cells - 1u), 0.0f, vt));
    } else {
        AddStraight(st, SearchStraight(st, SECTION_MM * (float)cells, vt, vt));
    }
    st->at_center = false;
}

// 袋小路の 180°(機体の TurnBack)
static void TurnBack(SearchTime *st, WallObservation w) {
    if (w.front && (w.right || w.left)) {
        AddPivot(st, 90.0f);
        AddSetPos(st);
        AddPivot(st, 90.0f);
        AddSetPos(st);
        return;
    }
    AddPivot(st, 180.0f);
}

// ゴール・スタートの 180°(機体の TurnBackAtGoal: 壁が一つでもあれば尻当て)
static void TurnBackAtGoal(SearchTime *st, WallObservation w) {
    if (w.front && (w.right || w.left)) {
        TurnBack(st, w);
    } else if (w.right || w.left) {
        AddPivot(st, 90.0f);
        AddSetPos(st);
        AddPivot(st, 90.0f);
    } else if (w.front) {
        AddPivot(st, 180.0f);
        AddSetPos(st);
    } else {
        AddPivot(st, 180.0f);
    }
}

static void GoalWait(SearchTime *st) {
    float t = (float)SEARCH_GOAL_WAIT_MS * 1e-3f;
    st->t_s += t;
    st->t_goal_wait += t;
}

// ---- 公開 ----

void SearchTime_Init(SearchTime *st, const SearchTimeParams *p) {
    SearchTime zero = { 0 };
    *st = zero;
    st->p = *p;
    st->t_goal_s = -1.0f;
    st->at_center = true;

    // スラローム1回(機体の ComputeSlalomOffsets と同じ形): 前のオフセット → 曲がる → 後ろのオフセット を同じ速さで
    SlalomParams sp = { SLALOM_V_MM_S, SLALOM_OMEGA_DPS, SLALOM_ALPHA_DPS2, 90.0f };
    Slalom_ScaleToSpeed(&sp, p->turn_v_mm_s);
    SlalomOffsets o = Slalom_SmallTurnOffsets(&sp);
    st->slalom_s = (o.pre_mm + Slalom_ComputeShape(&sp).length_mm + o.post_mm) / p->turn_v_mm_s;

    // 既知の区間の速さ(機体の ComputeSlalomOffsets: 直進は探索の直進とスラロームの速い方、加速度は SEARCH_KNOWN_ACCEL_MM_S2)
    float v = (p->v_mm_s > p->turn_v_mm_s) ? p->v_mm_s : p->turn_v_mm_s;
    st->known_prof = RunProfile_ForSpeeds(v, SEARCH_KNOWN_ACCEL_MM_S2, p->turn_v_mm_s, NULL);
    st->known_prof.decel = (SEARCH_KNOWN_ACCEL_MM_S2 < FAST_DECEL_MAX_MM_S2) ? SEARCH_KNOWN_ACCEL_MM_S2 : FAST_DECEL_MAX_MM_S2;
    st->known_large = SEARCH_KNOWN_LARGE && p->v_mm_s >= FAST_LARGE90_V_MM_S && p->v_mm_s >= FAST_LARGE180_V_MM_S;
}

void SearchTime_Start(SearchTime *st) {
    AddPivot(st, 90.0f);
    AddSetPos(st);
    AddPivot(st, 90.0f);
    AddSetPos(st);
    st->at_center = true;
}

// 指令の i 番目の前後の速さ・オフセット(旋回でなければ 0)
static float CmdSpeed(const RunProfile *p, const RunList *l, int i) {
    if (i < 0 || i >= (int)l->count || !RunType_IsTurn((RunType)l->items[i].type)) return 0.0f;
    return p->v_turn[l->items[i].type];
}
static float CmdPre(const RunProfile *p, const RunList *l, int i) {
    if (i < 0 || i >= (int)l->count || !RunType_IsTurn((RunType)l->items[i].type)) return 0.0f;
    return p->turn_pre[l->items[i].type];
}
static float CmdPost(const RunProfile *p, const RunList *l, int i) {
    if (i < 0 || i >= (int)l->count || !RunType_IsTurn((RunType)l->items[i].type)) return 0.0f;
    return p->turn_post[l->items[i].type];
}

float SearchTime_KnownRunTime(const SearchTime *st, const RunList *list) {
    // 機体の RunList_Drive(v_final = スラロームの速さ)。指令の列は1つ手前の区画の真ん中から作ってあるが、機体は
    // その先の境界をスラロームの速さで通っている所から走るので、最初の直進は半区画短く、入りの速さはスラロームの速さ。
    // 最後の直進は止まらずにスラロームの速さで終わりの境界へ入る。
    const RunProfile *p = &st->known_prof;
    float vt = st->p.turn_v_mm_s;
    float total = 0.0f;
    int n = (int)list->count;
    while (n > 0 && list->items[n - 1].type == RUN_STOP) n--;
    for (int i = 0; i < n; i++) {
        RunType t = (RunType)list->items[i].type;
        float dt = 0.0f;
        if (t == RUN_STRAIGHT) {
            float dist = (float)list->items[i].halves * HALF_SECTION_MM - ((i == 0) ? HALF_SECTION_MM : 0.0f);
            float v_in = (i == 0) ? vt : CmdSpeed(p, list, i - 1);
            float v_out = (i == n - 1) ? vt : CmdSpeed(p, list, i + 1);
            RunProfile_LinkedStraightTime(p, dist, v_in, CmdPost(p, list, i - 1), v_out,
                                          (i == n - 1) ? 0.0f : CmdPre(p, list, i + 1), &dt);
        } else if (RunType_IsTurn(t)) {
            dt = RunProfile_CurveTime(p, t);
            if (i > 0 && RunType_IsTurn((RunType)list->items[i - 1].type)) {
                float link = 0.0f;
                RunProfile_LinkedStraightTime(p, 0.0f, CmdSpeed(p, list, i - 1), CmdPost(p, list, i - 1),
                                              p->v_turn[t], p->turn_pre[t], &link);
                dt += link;
            }
        }
        total += dt;
    }
    return total;
}

void SearchTime_Action(SearchTime *st, Action a, WallObservation walls, SearchPhase phase_after, const RunList *known,
                       uint16_t known_cells) {
    if (st->finished) return;
    if (phase_after != SEARCH_PHASE_FULL && phase_after != SEARCH_PHASE_FAILED && st->t_goal_s < 0.0f &&
        st->p.scope == SEARCH_TIME_FULL) {
        st->t_goal_s = st->t_s; // 全面探索: 最短経路が決まった(この指令からスタートへ戻る)
    }
    switch ((ActionType)a.type) {
        case ACTION_FORWARD:
            if (known != NULL) {
                float t = SearchTime_KnownRunTime(st, known);
                st->t_s += t;
                st->t_known += t;
                st->n_known++;
                st->n_known_cells = (uint16_t)(st->n_known_cells + known_cells);
                st->at_center = false;
            } else {
                GoToNextBoundary(st, a.cells);
            }
            break;
        case ACTION_TURN_RIGHT:
        case ACTION_TURN_LEFT:
            if (st->p.slalom && !st->at_center) {
                st->t_s += st->slalom_s;
                st->t_slalom += st->slalom_s;
            } else {
                StopAtCenter(st);
                AddPivot(st, 90.0f);
                GoToNextBoundary(st, 1);
            }
            break;
        case ACTION_TURN_BACK:
            StopAtCenter(st);
            TurnBack(st, walls);
            GoToNextBoundary(st, 1);
            break;
        case ACTION_STOP:
        default:
            StopAtCenter(st);
            if (phase_after == SEARCH_PHASE_FAILED) {
                st->finished = true;
                break;
            }
            if (st->t_goal_s < 0.0f) st->t_goal_s = st->t_s; // 往復・片道: ゴールの真ん中に止まった
            if (st->p.scope == SEARCH_TIME_ONE_WAY && phase_after == SEARCH_PHASE_TO_START) {
                GoalWait(st); // 片道: ゴールで待って終わる
                st->finished = true;
                break;
            }
            TurnBackAtGoal(st, walls);
            GoalWait(st);
            if (phase_after == SEARCH_PHASE_DONE) st->finished = true;
            break;
    }
}

void SearchTime_Step(SearchTime *st, const SearchPlanner *sp, WallObservation obs, Action a, SearchPhase phase_before) {
    (void)phase_before;
    if (st->finished) return;
    if (st->skip > 0) { // 既知の区間としてまとめて走った区画(時間は数えてある)
        st->skip--;
        return;
    }
    if (a.type == ACTION_FORWARD && SEARCH_KNOWN_FAST_ENABLE && !st->at_center) {
        // 機体の TryKnownRun と同じ先読み(Step で直進を返した直後のプランナー)
        static Direction moves[MAZE_CELL_COUNT];
        static RunList list;
        uint16_t m = SearchPlanner_KnownRun(sp, SEARCH_KNOWN_MIN_MOVES, moves);
        if (m > 0u && RunPath_FromKnownRun(moves, m, &st->known_prof, st->known_large, &list)) {
            SearchTime_Action(st, a, obs, sp->phase, &list, (uint16_t)(m + 1u));
            st->skip = m;
            return;
        }
    }
    SearchTime_Action(st, a, obs, sp->phase, NULL, 0);
}
