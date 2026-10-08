#ifndef INC_PRIORITYQUEUE_H_
#define INC_PRIORITYQUEUE_H_


#include "global.h"
#include "logic/maze/maze_types.h"

// Dijkstra用の優先度付きキュー(最小ヒープ、mallocなし)。
// ノード番号(0〜MAZE_NODE_COUNT-1)を入れ、コストが最小のものから取り出す。
// コストは呼び出し側のノードの状態(MazeNodeState.f.cost)を直接読む(キューにはコピーしない)。
//
// BlueEyesは同じノードを何度も積む方式で、キューが満杯になると積めずに捨てていた。
// ここでは各ノードがヒープのどこにあるか(pos)を覚えておき、コストが下がったら
// その場で位置を直す(decrease-key)。同じノードは1つしか入らないので、
// ノード数ぶんの大きさで必ず足りる。大きさは4KB(heap 2KB + pos 2KB)。
#define PQ_CAPACITY MAZE_NODE_COUNT
#define PQ_NOT_IN_QUEUE 0xFFFFu

typedef struct {
    uint16_t heap[PQ_CAPACITY]; // ヒープ(ノード番号)
    uint16_t pos[PQ_CAPACITY];  // ノード番号 → heap内の位置(入っていなければPQ_NOT_IN_QUEUE)
    uint16_t size;
    const MazeNodeState *state; // ノード番号 → 状態(コストを読む)
} PriorityQueue;

// 空にする。stateはMAZE_NODE_COUNT個のノードの状態の配列。
void PQ_Init(PriorityQueue *q, const MazeNodeState *state);

// state[node]のコストを下げた後に呼ぶ。入っていなければ入れ、入っていれば位置を直す。
// コストは下げるだけにすること(上げると順序が崩れる)。
void PQ_PushOrUpdate(PriorityQueue *q, uint16_t node);

bool PQ_IsEmpty(const PriorityQueue *q);

// コスト最小のノードを取り出す。空ならMAZE_NODE_NONE。
uint16_t PQ_Pop(PriorityQueue *q);

#endif
