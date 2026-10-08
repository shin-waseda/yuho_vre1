#include "logic/maze/bucket_queue.h"

static uint8_t BucketOf(const BucketQueue *q, uint16_t node) {
    return (uint8_t)(q->state[node].f.cost % BQ_BUCKETS);
}

static void Unlink(BucketQueue *q, uint16_t node) {
    uint8_t b = q->bucket[node];
    uint16_t p = q->prev[node];
    uint16_t n = q->next[node];
    if (p == MAZE_NODE_NONE) {
        q->head[b] = n;
    } else {
        q->next[p] = n;
    }
    if (n != MAZE_NODE_NONE) q->prev[n] = p;
    q->bucket[node] = BQ_NOT_IN_QUEUE;
    q->size--;
}

void BQ_Init(BucketQueue *q, const MazeNodeState *state) {
    for (uint16_t i = 0; i < BQ_BUCKETS; i++) {
        q->head[i] = MAZE_NODE_NONE;
    }
    for (uint16_t i = 0; i < MAZE_NODE_COUNT; i++) {
        q->bucket[i] = BQ_NOT_IN_QUEUE;
    }
    q->size = 0;
    q->cur = 0;
    q->state = state;
}

void BQ_PushOrUpdate(BucketQueue *q, uint16_t node) {
    if (node >= MAZE_NODE_COUNT) return;
    if (q->bucket[node] != BQ_NOT_IN_QUEUE) Unlink(q, node);
    uint8_t b = BucketOf(q, node);
    q->prev[node] = MAZE_NODE_NONE;
    q->next[node] = q->head[b];
    if (q->head[b] != MAZE_NODE_NONE) q->prev[q->head[b]] = node;
    q->head[b] = node;
    q->bucket[node] = b;
    q->size++;
}

bool BQ_IsEmpty(const BucketQueue *q) {
    return q->size == 0;
}

uint16_t BQ_Pop(BucketQueue *q) {
    if (q->size == 0) return MAZE_NODE_NONE;
    // 中身は「cur〜cur + BQ_MAX_STEP」にあるので、BQ_BUCKETS 個見るまでに必ず見つかる
    while (q->head[q->cur % BQ_BUCKETS] == MAZE_NODE_NONE) {
        q->cur++;
    }
    uint16_t node = q->head[q->cur % BQ_BUCKETS];
    Unlink(q, node);
    return node;
}
