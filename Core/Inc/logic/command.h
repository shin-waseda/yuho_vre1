#ifndef INC_COMMAND_H_
#define INC_COMMAND_H_


#include "global.h"
#include "params.h"

// 走行の指令(アクションコマンド)。logic層(探索・経路計算)が作り、
// 実機ではapp層が走行関数で、PCではシミュレータ(tools/maze_sim)が座標を動かして実行する。
// どの指令も「今いる区画から、隣(FORWARDはcells先)の区画へ移る」単位で表す。
// 旋回をその場で回るか(超信地)、曲がりながら進むか(スラローム)は実行する側が決める。
typedef enum {
    ACTION_STOP = 0,   // 止まる(経路の終わり・ゴールやスタートに着いた)
    ACTION_FORWARD,    // 向きを変えずに cells 区画進む
    ACTION_TURN_RIGHT, // 右へ90°向きを変えて、隣の区画へ進む
    ACTION_TURN_LEFT,  // 左へ90°向きを変えて、隣の区画へ進む
    ACTION_TURN_BACK,  // 180°向きを変えて、後ろの区画へ進む
} ActionType;

// 2バイト。typeはActionTypeの値(enumのままだと4バイトになるのでuint8_tで持つ)。
typedef struct {
    uint8_t type;  // ActionType
    uint8_t cells; // 進む区画数(FORWARDは1以上、旋回は1、STOPは0)
} Action;

// 経路(指令の列)。1区画ずつでも全区画を通れる長さ + 終端のSTOP。約0.5KB。
#define COMMAND_LIST_MAX (MAZE_SIZE * MAZE_SIZE + 1)

typedef struct {
    Action items[COMMAND_LIST_MAX];
    uint16_t count;
} CommandList;

void CommandList_Clear(CommandList *list);

// 末尾に追加する。merge_forwardがtrueで、直前もFORWARDなら区画数を足して1つにまとめる。
// 満杯ならfalse。
bool CommandList_Push(CommandList *list, Action action, bool merge_forward);

// 指令で向きが何回(90°単位、時計回りが正)変わるか。右+1、左-1、後ろ+2、それ以外0。
int Action_QuarterTurns(ActionType type);

// 向きを時計回りにquarter_cw × 90°変えて隣の区画へ進む指令(Action_QuarterTurnsの逆)。
// 0: FORWARD x1、1: RIGHT、2: BACK、3(-1): LEFT。
Action Action_Move(int quarter_cw);

const char *Action_Name(ActionType type);

// 経路を1行1指令でprintfする(デバッグ用)。
void CommandList_Print(const CommandList *list);

#endif
