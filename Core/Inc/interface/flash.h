#ifndef INC_FLASH_H_
#define INC_FLASH_H_


#include "global.h"
#include "params.h"

// マイコンの flash の最後のセクタ(セクタ11、0x080E0000〜、128KB)に、電源を切っても残るデータを1つ置く。
// 迷路の地図(探索の結果)を最短走行のモードへ渡すのに使う(BlueEyes の store_map_in_flash と同じ使い方)。
// プログラムは先頭から置かれ(今は約110KB)、このセクタまでは届かない前提。
//
// 書き込みはセクタを消してから書くので 1〜2 秒かかり、その間は flash からの命令の読み出しが止まる
// (割り込みも待たされる)。制御を無効にし、モーターを止めているときに呼ぶこと。

// data を len バイト書く(前の内容は消える)。書いた後に読み直して確かめる。失敗なら false。
bool Flash_WriteUserData(const void *data, uint32_t len);

// 書いてあるデータを len バイト読む。書いた時と長さが違う・壊れている・何も書いていなければ false。
bool Flash_ReadUserData(void *data, uint32_t len);

#endif
