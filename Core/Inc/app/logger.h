#ifndef INC_LOGGER_H_
#define INC_LOGGER_H_


#include "global.h"
#include "params.h"

// divergence_v3 の Logger を C へ移植したもの。
// 1kHzのISRでLogger_Sample()を呼び、登録した列(float)をRAMのバッファへ記録する。
// 止めた後にLogger_Dump()でPCへ送る。PC側は tools/get_log.py で受けてCSVにする。
//
// 送信形式(divergence_v3と同じ):
//   BIN_START\r\n
//   <dir>\r\n
//   <file>\r\n            (空ならPC側で "log")
//   TIMESTAMP:<0|1>\r\n   (1ならPC側でファイル名に日時を付ける)
//   SIZE:<bytes>\r\n
//   <name0>,<name1>,...\r\n
//   <float32(リトルエンディアン) × 列数 × 行数 の生データ>
//   BIN_END\r\n
//
// 先頭の列は常に "time_s"(記録開始からの秒)。

#define LOGGER_MAX_FIELDS  16
// 8列なら1500サンプル(1kHzで1.5s)。長い記録はLogger_SetDuration()で間引く。48KB。
#define LOGGER_BUFFER_SIZE (8u * 1500u)

// 列とバッファを空にし、先頭に "time_s" 列を置く。保存先は "." / "log"、日時付き。
// 記録中に呼ばないこと。
void Logger_Init(void);

// 列を追加する。valueはISRが書き換える変数(App_GetControlDebug()のメンバ等)。
// 満杯ならfalse。記録中に呼ばないこと。
bool Logger_AddField(const char *name, const volatile float *value);

void Logger_SetDirName(const char *name);
void Logger_SetFileName(const char *name);
void Logger_SetIncludeTimestamp(bool enable);

// n tickに1回だけ記録する(既定1=毎tick)。Logger_SetDuration()を取り消す。
void Logger_SetDecimation(uint32_t every_n_ticks);
// 少なくとも duration_ms 記録できるよう、Start時の列数から間引きを決める。
// Logger_SetDecimation()を取り消す。
void Logger_SetDuration(uint32_t duration_ms);
// 今の列数・間引きで記録できる長さ[ms](Start後に確認用)。
uint32_t Logger_RecordableMs(void);

void Logger_Start(void);
void Logger_Stop(void);

// ISR(1kHz)から毎tick呼ぶ。記録中でなければ何もしない。
void Logger_Sample(void);

// 止めた状態でのみ送る(記録中はISRと取り合うので送らない)。ブロッキング。
void Logger_Dump(void);

bool Logger_IsRecording(void);
bool Logger_IsFull(void);
uint32_t Logger_SampleCount(void);
uint32_t Logger_FieldCount(void);

#endif
