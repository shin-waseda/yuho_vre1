#ifndef INC_SDCARD_H_
#define INC_SDCARD_H_


#include "global.h"
#include "params.h"

// microSDカード(SDIO 4bit + DMA, FatFs)。
// 失敗してもError_Handler()では止めず、falseを返す(カードなしでも機体は動かす)。
// 同時に開けるファイルは1つだけ。メインループから呼ぶこと(ISRからは呼ばない)。

// SDIOの設定(MX_SDIO_SD_Init)→マウントを行う。起動時に1回呼ぶ。
// カードがないと、HALの初期化のタイムアウト待ちで数秒かかることがある。
bool SDCard_Mount(void);
bool SDCard_IsMounted(void);

// パスの最大長(終端含む)。"dir/prefix_NNNN.csv" が収まる長さ。
#define SDCARD_PATH_MAX 64
// 送信済みのファイルを移すフォルダ。sent/<元のパス> に置く。
#define SDCARD_SENT_DIR "sent"

// dir/prefix_NNNN.ext を、まだ使っていない番号(0001〜9999)で新しく作って開く。
// sent/ へ移動済みの番号も使用済みとみなす。dirがなければ作る。
// 開いたパスをpath_out(NULL可)へ書く。
bool SDCard_OpenNewSequential(const char *dir, const char *prefix, const char *ext,
                              char *path_out, uint32_t path_len);

bool SDCard_Write(const void *data, uint32_t len);

// 既存のファイルを読み込み用に開く。size_out(NULL可)にファイルサイズを書く。
bool SDCard_OpenRead(const char *path, uint32_t *size_out);
// 最大len バイト読む。読めたバイト数をgot_out(NULL可)へ。終端ならgot=0でtrue。
bool SDCard_Read(void *buf, uint32_t len, uint32_t *got_out);

// 書いた内容をカードへ確定させる(開いたまま)。途中で電源が切れても、ここまでは残る。
bool SDCard_Sync(void);

bool SDCard_Close(void);

// base("" ならルート)直下と、その1段下のフォルダにあるログ(.csv と .bin)のパスを、
// 見つけた順に先頭からskip個読み飛ばして、最大max個集める。sent/・隠しフォルダは除く。
// パスは base を含む形(例: base="sent" なら "sent/straight/xxx.csv")。
// 集めた数を返す(baseがなければ0、失敗時は-1)。max未満なら最後まで読んだ。
int SDCard_ListCsv(const char *base, int skip, char (*paths)[SDCARD_PATH_MAX], int max);

// 未送信のCSV(= ListCsv("", 0, ...))。
int SDCard_ListUnsent(char (*paths)[SDCARD_PATH_MAX], int max);

// path を sent/<path> へ移動する(送信済みの印)。ファイルは消えない。
bool SDCard_MoveToSent(const char *path);

#endif
