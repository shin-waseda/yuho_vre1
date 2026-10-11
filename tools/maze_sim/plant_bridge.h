#ifndef PLANT_BRIDGE_H_
#define PLANT_BRIDGE_H_

// maze_sim から plant_sim(実機のファームウェアを PC で動かし、車輪・ジャイロ・壁センサーまで再現するシミュ。
// sim の worktree の tools/plant_sim)を呼び、同じ迷路・同じ探索の設定で探索を走らせて、機体と同じ形式のログを残す。
//   <out>/search/search_NNNN.bin   機体の SD と同じログ(YLOG2)。tools/get_log.py --bin2csv で同じ名前の .csv も作る
//   <out>/truth.csv                plant_sim の本当の状態(10ms ごと)
//   <out>/flash.bin                探索で残した地図(plant_sim の --flash。この後に最短走行を走らせるとき用)
//   <out>/plant_stdout.txt / plant_stderr.txt   ファームウェアの printf / plant_sim の表示
// 機体の操作(RUN → SEARCH → MAP 1 → SCOPE → ALGO → SPEED → ACCEL → SLALOM → TURN → 手かざし)は、params.h の
// SPEED_SELECT_* の表から作る。選べる値でなければ走らせない。
//
// plant_sim の場所は、--plant-exe、環境変数 YUHO_PLANT_SIM、tools/maze_sim/plant_sim.local の1行目(git に入れない)の順に探す。
// plant_sim は yuho のファームウェア(plant_sim の fw_root.local)をコンパイルしたものなので、走らせる前に plant_sim の
// build.sh でビルドし直す(古いビルドのまま走らせないため。--plant-no-build で省く)。

#include <stdbool.h>
#include <stddef.h>

#include "search_time.h"
#include "logic/maze/search_planner.h"

typedef struct {
    const char *exe;      // plant_sim の実行ファイル(NULL なら環境変数・plant_sim.local)
    bool build;           // 走らせる前に plant_sim の build.sh を実行する
    const char *extra;    // plant_sim にそのまま渡すオプション(例 "--no-crash-stop"。NULL 可)
    const char *self;     // maze_sim 自身のパス(argv[0]。plant_sim.local と tools/get_log.py を探すのに使う)
} PlantOptions;

// 走らせてログを out_dir に残す。est_s は maze_sim の探索の時間の見積もり(打ち切りの時間を決めるのと、比べて表示するのに使う)。
// 成功したら true(plant_sim が壁に当たって止まったときも、ログが残れば true。結果は表示する)。
// log_out が NULL でなければ、できたログのパス(.csv。作れなければ .bin)を書く(失敗したら空)。
bool PlantLog_Run(const PlantOptions *o, const char *maze_path, const SearchTimeParams *sp, SearchAlgo algo,
                  float est_s, const char *out_dir, char *log_out, size_t log_len);

#endif
