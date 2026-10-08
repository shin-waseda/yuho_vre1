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

#define LOGGER_MAX_FIELDS  40 // time_s を含む(全部の値 + イベントの6列を残すため)

// 走りながら SD へ流すときのブロックの大きさ(SD のセクタ 512 の倍数)。バッファはこれを2つ持てる大きさ。
#define LOGGER_STREAM_BLOCK_BYTES   24576u
// 流すときに、ファイルを先に確保しておく大きさ(15列・5ms ごとなら約 11 分ぶん)
#define LOGGER_STREAM_RESERVE_BYTES (8u * 1024u * 1024u)

// RAM に貯める記録のバッファ[float]。8列なら約1500サンプル(1kHzで1.5s)。
// 長い記録はLogger_SetDuration()で間引く。48KB。
#define LOGGER_BUFFER_SIZE (2u * LOGGER_STREAM_BLOCK_BYTES / 4u)

// 列とバッファを空にし、先頭に "time_s" 列を置く。保存先は "." / "log"、日時付き。
// 記録中に呼ばないこと。
void Logger_Init(void);

// 列を追加する。valueはISRが書き換える変数(App_GetControlDebug()のメンバ等)。
// 満杯ならfalse。記録中に呼ばないこと。
bool Logger_AddField(const char *name, const volatile float *value);

// イベントの列("ev" と中身の "ev_a"〜"ev_e")を足す。イベントを残すログで、Logger_AddField の後に呼ぶ。
void Logger_AddEventFields(void);

// イベントを残す(番号は app/log_event.h の LogEventCode、中身は最大 5 つ)。記録している間だけ残る。
// 次に記録する行の "ev" 列に入る(1行に1つ。続けて入れたものは次の行から順に)。メインからも ISR からも呼べる。
// 順番待ちがいっぱいなら捨てて数える(Logger_EventDropped)。
void Logger_Event(uint16_t code, float a, float b, float c, float d, float e);
uint32_t Logger_EventDropped(void);
// 順番待ちのイベントの数と、順番待ちに入れられる最大の数(続けて沢山入れるときに、空くのを待つため)
uint32_t Logger_EventQueued(void);
uint32_t Logger_EventCapacity(void);

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

// SD に書くファイルは、すべてバイナリ(<dir>/<file>_NNNN.bin、中身は logger.c の LOG_BIN_MAGIC の説明)。
// 数値を文字にしないので CSV より速い。SD_DUMP で PC へ送ると、tools/get_log.py が同じ名前の .csv も作る。
// dir/file は Logger_SetDirName/SetFileName(file が未設定なら "log")。番号は空いているものを使い、上書きしない。

// 止めた状態でのみ、記録をSDカードへ1つのファイルとして保存する。ブロッキング。
// 保存したパスをpath_out(NULL可)へ書く。SD未マウント・失敗時はfalse。
bool Logger_SaveFile(char *path_out, uint32_t path_len);

// ---- 走りながら SD へ流し続ける(探索・最短走行など、長い走行用) ----
// 使い方: Logger_StreamBegin() → Logger_Start() → (待ちのループの中で何度も) Logger_StreamPoll()
//         → Logger_StreamEnd()
// バッファを 24KB のブロック2つに分け、ISR が片方に記録している間に、もう片方を SD へ送る
// (SD は待たない書き込み。送り出して、終わったかを見に行くだけ)。止まらずに記録し続けられる。
// SD が長く待たせて両方のブロックがいっぱいになったら、その間の行は捨てて数える。
// 書く形式は YLOG2(logger.c の説明)。流している間は他のファイルを開けない。

// 新しいファイルを作って先を確保し、先頭を書く(走る前に呼ぶ。確保に時間がかかることがある)。
bool Logger_StreamBegin(char *path_out, uint32_t path_len);
// いっぱいになったブロックを SD へ送り出す・送り終わったかを見る。待たない。
// 記録している間、メインの待ちのループの中で何度も呼ぶこと(呼ばないと SD へ送られず、行が捨てられる)。
void Logger_StreamPoll(void);
// 記録を止め、残りを送ってファイルを閉じる(止まっているときに呼ぶ)。dropped_rows(NULL可)に
// 捨てた行の数を書く。失敗なら false。
bool Logger_StreamEnd(uint32_t *dropped_rows);
// 流している途中で SD への書き込みに失敗したら true(以後は書かない)。
bool Logger_StreamFailed(void);

// 流す方式の統計(確かめる用)。Logger_StreamEnd の後に呼んでも、その回の値が残っている。
typedef struct {
    uint32_t blocks;         // SD へ書き終えたブロックの数
    uint32_t dropped_rows;   // 両方のブロックがいっぱいで捨てた行の数
    uint32_t write_max_ms;   // 1ブロックを送るのに一番かかった時間
    uint32_t write_avg_ms;   // その平均
    uint32_t rows_per_block; // 1ブロックに入る行の数
} LoggerStreamStats;

void Logger_StreamGetStats(LoggerStreamStats *st);

bool Logger_IsRecording(void);
bool Logger_IsFull(void);
uint32_t Logger_SampleCount(void);
uint32_t Logger_FieldCount(void);

#endif
