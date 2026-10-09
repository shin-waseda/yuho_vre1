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

// RUN の SEARCH。地図(1 初期化 / 2 flash の地図に重ねる)・行き先(1 往復 / 2 片道 / 3 全面(まだ作っていない))・
// アルゴリズム(1 Dijkstra / 2 足立法)・直進の速さ・加速度・スラロームの速さ・曲がり方(1 スラローム / 2 超信地旋回)を
// 選んでから、手かざしで探索する。片道はゴールの真ん中で止まって終わる(地図は flash に残す)。電源を切るまで戻らない。
void SearchMenu_Run(void);

// SearchRun_Run(Dijkstra)と同じ探索で、初めて入った区画ごとに真ん中で止まり、左に1周・右に1周回る
// (壁センサーのモデル用。回り方は params.h の SENSOR_SPIN_*)。区画と壁は LOG_EV_SENSOR_SPIN でログに残る。
// ログは SD の search/spin_NNNN。既知の区間をまとめて走るのは使わない。電源を切るまで戻らない。
void SearchSpin_Run(void);

// 最短走行。探索で flash に残した地図の、分かっている壁だけで経路を計算し、
// スタートの真ん中からゴールまで走る(続く直進はまとめて速く走り、曲がるときは止まって超信地旋回)。
// 地図がない・経路がないときは、全部の直結 LED を点滅させて止まる。
// 電源を切るまで戻らない。
void FastRun_Run(void);

// start_sequence(BlueEyes と同じ。右90° → 尻当て → 左90° → 尻当て)で、区画の真ん中に北向き(置いた向き)に
// そろえる。左と後ろに壁がある区画に置き、制御を有効にしてから呼ぶこと(試験モードからも使う)。
// 尻当てで制御を有効にし直すので、終わった時点の目標の距離・向きが新しい基準になる。打ち切ったら false。
bool MazeRun_StartSequence(void);

// 長い走行(TEST の LONG_LOG)。ログを取るために、探索を「直進の速さ × スラロームの速さ」の組み合わせで、
// 次に最短走行を小回り(小回りの速さごと)と大回りで、LONG_LOG_REPEAT 回ずつ続けて走る。最短走行の後はゴールから
// 自分でスタートへ戻るので、置き直さずに続けられる。走る前に、どこから始めるか(段と番号)を選び、最初の1回だけ
// 手かざしで始める。電池が LONG_LOG_MIN_VBAT_V より下がったら、次に走る段と番号を LED で出して止まる。
// 電源を切るまで戻らない。
void LongLogRun_Run(void);

// 最短走行の連続(RUN の FAST_SWEEP)。探索で flash に残した地図で、直進の速さ・加速度・小回りの速さの範囲(FROM / TO)と
// 走り方(1 SMALL / 2 LARGE / 3 両方)を選び、1回の手かざしで全部の組み合わせを FAST_SWEEP_REPEAT 回ずつ走る。
// 1本ごとにゴールからスタートへ自分で戻る(置き直さない)。打ち切り・電池の低下・全部終わったら、何本目かを
// LED の棒グラフで点滅させて止まる。電源を切るまで戻らない。
void FastSweep_Run(void);

// 速度帯の最短走行(RUN の FAST_BANDS)。params.h の FAST_BANDS から始めと終わりの速度帯を選び、1回の手かざしで、
// 遅い速度帯から順に「最短走行 → ゴールからスタートへ自分で戻る」を FAST_BAND_REPEAT 回ずつ走る。
// ログ取りにも本番(始めと終わりに同じ速度帯)にも使う。走り始めに LOG_EV_FAST_BAND を入れる。電源を切るまで戻らない。
void FastBands_Run(void);

#endif
