#include "logic/maze/priority_queue.h"

// ヒープのi番目のノードのコスト
#define KEY(q, i) ((q)->state[(q)->heap[i]].f.cost)

static void Swap(PriorityQueue *q, uint16_t i, uint16_t j) {
    uint16_t t = q->heap[i];
    q->heap[i] = q->heap[j];
    q->heap[j] = t;
    q->pos[q->heap[i]] = i;
    q->pos[q->heap[j]] = j;
}

static void SiftUp(PriorityQueue *q, uint16_t i) {
    while (i > 0) {
        uint16_t parent = (uint16_t)((i - 1u) / 2u);
        if (KEY(q, parent) <= KEY(q, i)) break;
        Swap(q, parent, i);
        i = parent;
    }
}

static void SiftDown(PriorityQueue *q, uint16_t i) {
    while (1) {
        uint32_t l = 2u * i + 1u;
        uint32_t r = 2u * i + 2u;
        uint16_t smallest = i;
        if (l < q->size && KEY(q, l) < KEY(q, smallest)) smallest = (uint16_t)l;
        if (r < q->size && KEY(q, r) < KEY(q, smallest)) smallest = (uint16_t)r;
        if (smallest == i) break;
        Swap(q, i, smallest);
        i = smallest;
    }
}

void PQ_Init(PriorityQueue *q, const MazeNodeState *state) {
    q->size = 0;
    q->state = state;
    for (uint16_t i = 0; i < PQ_CAPACITY; i++) {
        q->pos[i] = PQ_NOT_IN_QUEUE;
    }
}

void PQ_PushOrUpdate(PriorityQueue *q, uint16_t node) {
    if (node >= PQ_CAPACITY) return;
    if (q->pos[node] == PQ_NOT_IN_QUEUE) {
        // 同じノードは1つしか入らないので、sizeがPQ_CAPACITYを超えることはない
        uint16_t i = q->size++;
        q->heap[i] = node;
        q->pos[node] = i;
        SiftUp(q, i);
    } else {
        SiftUp(q, q->pos[node]);
    }
}

bool PQ_IsEmpty(const PriorityQueue *q) {
    return q->size == 0;
}

uint16_t PQ_Pop(PriorityQueue *q) {
    if (q->size == 0) return MAZE_NODE_NONE;

    uint16_t top = q->heap[0];
    q->pos[top] = PQ_NOT_IN_QUEUE;
    q->size--;
    if (q->size > 0) {
        q->heap[0] = q->heap[q->size];
        q->pos[q->heap[0]] = 0;
        SiftDown(q, 0);
    }
    return top;
}
