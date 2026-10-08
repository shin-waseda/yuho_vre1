#include "logic/command.h"

#include <stdio.h>

void CommandList_Clear(CommandList *list) {
    list->count = 0;
}

bool CommandList_Push(CommandList *list, Action action, bool merge_forward) {
    if (merge_forward && action.type == ACTION_FORWARD && list->count > 0) {
        Action *last = &list->items[list->count - 1];
        if (last->type == ACTION_FORWARD && (uint16_t)last->cells + action.cells <= 0xFFu) {
            last->cells = (uint8_t)(last->cells + action.cells);
            return true;
        }
    }
    if (list->count >= COMMAND_LIST_MAX) return false;
    list->items[list->count++] = action;
    return true;
}

int Action_QuarterTurns(ActionType type) {
    switch (type) {
        case ACTION_TURN_RIGHT: return 1;
        case ACTION_TURN_LEFT:  return -1;
        case ACTION_TURN_BACK:  return 2;
        default:                return 0;
    }
}

Action Action_Move(int quarter_cw) {
    static const ActionType kByTurn[4] = {
        ACTION_FORWARD, ACTION_TURN_RIGHT, ACTION_TURN_BACK, ACTION_TURN_LEFT,
    };
    Action a = { (uint8_t)kByTurn[quarter_cw & 3], 1 };
    return a;
}

const char *Action_Name(ActionType type) {
    switch (type) {
        case ACTION_STOP:       return "STOP";
        case ACTION_FORWARD:    return "FORWARD";
        case ACTION_TURN_RIGHT: return "RIGHT";
        case ACTION_TURN_LEFT:  return "LEFT";
        case ACTION_TURN_BACK:  return "BACK";
        default:                return "?";
    }
}

void CommandList_Print(const CommandList *list) {
    for (uint16_t i = 0; i < list->count; i++) {
        const Action *a = &list->items[i];
        if (a->type == ACTION_FORWARD) {
            printf("%3u: %s x%u\r\n", (unsigned)i, Action_Name(a->type), (unsigned)a->cells);
        } else {
            printf("%3u: %s\r\n", (unsigned)i, Action_Name(a->type));
        }
    }
}
