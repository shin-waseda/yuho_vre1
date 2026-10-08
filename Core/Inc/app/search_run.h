#ifndef INC_SEARCHRUN_H_
#define INC_SEARCHRUN_H_


#include "global.h"
#include "params.h"
#include "logic/maze/search_planner.h" // SearchAlgo

// 探索走行。スタート区画の真ん中に北向きで置き、手かざしで始める。
// ゴールへ行き(着いたら少し止まる)、スタートへ戻って止まる。algo で Dijkstra か足立法かを選ぶ。
// 走る前に、曲がり方(PIVOT: 超信地旋回 / SMALL: 小回りのスラローム)をボタンのクリックで切り替えて選ぶ。
// ゴールまで行けたら、地図をマイコンの flash に残す(最短走行のモードで使う)。
// 区画ごとの記録(位置・向き・壁・センサーの値・指令)は RAM に貯め、走り終わってから
// 迷路の絵と一緒に UART へ出す(走っている間に printf すると待ちが出るため)。
// 電源を切るまで戻らない。
void SearchRun_Run(SearchAlgo algo);

// 最短走行。探索で flash に残した地図の、分かっている壁だけで経路を計算し、
// スタートの真ん中からゴールまで走る(続く直進はまとめて速く走り、曲がるときは止まって超信地旋回)。
// 地図がない・経路がないときは、全部の直結 LED を点滅させて止まる。
// 電源を切るまで戻らない。
void FastRun_Run(void);

// start_sequence(BlueEyes と同じ。右90° → 尻当て → 左90° → 尻当て)で、区画の真ん中に北向き(置いた向き)に
// そろえる。左と後ろに壁がある区画に置き、制御を有効にしてから呼ぶこと(試験モードからも使う)。
// 尻当てで制御を有効にし直すので、終わった時点の目標の距離・向きが新しい基準になる。打ち切ったら false。
bool MazeRun_StartSequence(void);

#endif
