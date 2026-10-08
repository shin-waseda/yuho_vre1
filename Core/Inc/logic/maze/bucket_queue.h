#ifndef INC_BUCKETQUEUE_H_
#define INC_BUCKETQUEUE_H_


#include "global.h"
#include "logic/maze/maze_types.h"

// Dijkstra用のバケツのキュー(Dialの方法、mallocなし)。使い方は priority_queue.h と同じ。
// コストの値ごとに箱を作り、コストを BQ_BUCKETS で割った余りの箱にノードをつなぐ。
// 取り出すときは、今のコストの箱から順に、空でない箱を探すだけ(ヒープのような並べ直しがない)。
//
// 使える条件: 1区画進むときに増えるコストの最大が BQ_MAX_STEP 以下であること
// (キューの中のコストが常に「今のコスト〜今のコスト + BQ_MAX_STEP」に収まり、箱が混ざらない)。
// 取り出すコストは小さくなっていかないこと(Dijkstraなら必ずそうなる)。
// 大きさは約5KB(next/prev 各2KB + 箱の番号 1KB + 箱の先頭)。
#define BQ_BUCKETS  64u
#define BQ_MAX_STEP (BQ_BUCKETS - 1u)
#define BQ_NOT_IN_QUEUE 0xFFu

typedef struct {
    uint16_t head[BQ_BUCKETS];          // 箱 → 先頭のノード(空ならMAZE_NODE_NONE)
    uint16_t next[MAZE_NODE_COUNT];     // 同じ箱の次のノード
    uint16_t prev[MAZE_NODE_COUNT];     // 同じ箱の前のノード
    uint8_t bucket[MAZE_NODE_COUNT];    // ノード → 入っている箱(入っていなければBQ_NOT_IN_QUEUE)
    uint16_t size;
    uint16_t cur;                       // 次に取り出すコストの下限
    const MazeNodeState *state;         // ノード番号 → 状態(コストを読む)
} BucketQueue;

// 空にする。stateはMAZE_NODE_COUNT個のノードの状態の配列。
void BQ_Init(BucketQueue *q, const MazeNodeState *state);

// state[node]のコストを下げた後に呼ぶ。入っていなければ入れ、入っていれば箱を移す。
void BQ_PushOrUpdate(BucketQueue *q, uint16_t node);

bool BQ_IsEmpty(const BucketQueue *q);

// コスト最小のノードを取り出す。空ならMAZE_NODE_NONE。
// 同じコストのノードの取り出す順は決まっていない(Dijkstra側で順によらない結果にすること)。
uint16_t BQ_Pop(BucketQueue *q);

#endif
