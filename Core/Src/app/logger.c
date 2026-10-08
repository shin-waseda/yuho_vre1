#include "app/logger.h"

#include <stdio.h>
#include <string.h>
#include "main.h" // HAL_GetTick
#include "interface/uart.h"
#include "interface/sdcard.h"

// 1回のLogger_Sample()(制御の1tick)の長さ[ms]。制御周期は1ms以上の前提。
#define LOGGER_TICK_MS ((uint32_t)(CONTROL_DT_S * 1000.0f + 0.5f))

typedef enum {
    LOGGER_IDLE,
    LOGGER_RECORDING,
    LOGGER_STOPPED,
} LoggerState;

typedef struct {
    const char *name;
    const volatile float *value;
} LoggerField;

static LoggerField s_fields[LOGGER_MAX_FIELDS];
static uint32_t s_field_count = 0;
// RAM に貯める方式ではそのまま記録に使い、流す方式では前半・後半を2つのブロックとして使う
static float s_buffer[LOGGER_BUFFER_SIZE] __attribute__((aligned(4)));

// 以下はメイン(Start/Stop)とISR(Sample)の両方が触る
static volatile LoggerState s_state = LOGGER_IDLE;
static volatile uint32_t s_sample_count = 0;
static volatile bool s_full = false;

static uint32_t s_max_samples = LOGGER_BUFFER_SIZE;
static uint32_t s_decimation = 1;
static uint32_t s_duration_ms = 0; // 0ならs_decimationをそのまま使う
static uint32_t s_tick_count = 0;
static float s_time_s = 0.0f;      // "time_s"列の値。Sample内で更新する

static const char *s_dir_name = ".";
static const char *s_file_name = NULL;
static bool s_include_timestamp = true;

// ---- イベント ----
// Logger_Event で入れたイベントを順番待ちにしておき、ISR が次に記録する行の "ev"〜"ev_e" 列に1つずつ入れる
// (イベントのない行は 0)。5ms ごとの記録なら 1 秒に 200 個まで入れられる。
#define EVENT_FIFO_SIZE 32u
#define EVENT_VALUES    6u  // 番号 + 中身 5 つ

static float s_ev_fifo[EVENT_FIFO_SIZE][EVENT_VALUES];
static volatile uint32_t s_ev_head = 0; // 次に入れる所(Logger_Event)
static volatile uint32_t s_ev_tail = 0; // 次に取り出す所(ISR)
static volatile uint32_t s_ev_dropped = 0;
static float s_ev_row[EVENT_VALUES];     // 今の行のイベント(列として記録する)

// ---- 流す方式(Logger_StreamBegin 〜 Logger_StreamEnd)の状態 ----
// ブロック(24KB)を2つ持ち、ISR は片方に行を足していき、いっぱいになったらもう片方へ移る。
// メイン(Logger_StreamPoll)は、いっぱいになったブロックを SD へ待たずに送り出す。
// 送っている最中のブロックにはISRは書かない。両方いっぱいなら、その行は捨てて数える。
#define STREAM_BLOCK_BYTES   LOGGER_STREAM_BLOCK_BYTES
#define STREAM_BLOCK_FLOATS  (STREAM_BLOCK_BYTES / 4u)
#define STREAM_BLOCK_HEAD    4u // ブロックの頭の 4 語(magic, 通し番号, 行数, 列数)
#define STREAM_BLOCK_MAGIC   0x4B4C4259u // "YBLK"(リトルエンディアンで並べた文字)

_Static_assert(LOGGER_BUFFER_SIZE >= 2u * STREAM_BLOCK_FLOATS, "logger buffer must hold two stream blocks");
_Static_assert((STREAM_BLOCK_BYTES % 512u) == 0, "stream block must be a multiple of the SD sector");

static bool s_stream = false;                 // 流す方式で記録している(Begin 〜 End)
static volatile uint8_t s_blk_active = 0;     // ISR が書いているブロック
static volatile uint32_t s_blk_rows[2];       // ブロックごとの行数
static volatile bool s_blk_full[2];           // いっぱい(SD へ送る順番待ち・送っている最中)
static uint32_t s_blk_max_rows = 0;
static volatile uint32_t s_stream_dropped = 0; // 両方いっぱいで捨てた行の数
static uint8_t s_next_write = 0;              // 次に SD へ送るブロック(ISR が埋めた順 = 交互)
static int8_t s_writing = -1;                 // SD へ送っている最中のブロック(なければ -1)
static uint32_t s_blk_seq = 0;                // SD へ書き終えたブロックの数(ファイルの中の位置に使う)
static bool s_stream_failed = false;
static uint32_t s_last_poll_ms = 0;
static uint32_t s_write_start_ms = 0;   // 今のブロックを送り始めた時刻
static uint32_t s_write_max_ms = 0;     // 1ブロックを送るのに一番かかった時間
static uint32_t s_write_total_ms = 0;   // 合計(平均を出す用)

static float *BlockBase(uint8_t b) {
    return &s_buffer[(uint32_t)b * STREAM_BLOCK_FLOATS];
}

void Logger_Init(void) {
    s_state = LOGGER_IDLE;
    s_field_count = 0;
    s_max_samples = LOGGER_BUFFER_SIZE;
    s_decimation = 1;
    s_duration_ms = 0;
    s_sample_count = 0;
    s_full = false;
    s_dir_name = ".";
    s_file_name = NULL;
    s_include_timestamp = true;
    s_stream = false;
    Logger_AddField("time_s", &s_time_s);
}

bool Logger_AddField(const char *name, const volatile float *value) {
    if (s_field_count >= LOGGER_MAX_FIELDS) return false;
    if (name == NULL || value == NULL) return false;

    s_fields[s_field_count].name = name;
    s_fields[s_field_count].value = value;
    s_field_count++;
    s_max_samples = LOGGER_BUFFER_SIZE / s_field_count;
    s_state = LOGGER_IDLE;
    s_full = false;
    return true;
}

void Logger_AddEventFields(void) {
    static const char *const kNames[EVENT_VALUES] = { "ev", "ev_a", "ev_b", "ev_c", "ev_d", "ev_e" };
    for (uint32_t i = 0; i < EVENT_VALUES; i++) {
        Logger_AddField(kNames[i], &s_ev_row[i]);
    }
}

void Logger_Event(uint16_t code, float a, float b, float c, float d, float e) {
    if (s_state != LOGGER_RECORDING) return;
    // メインからも ISR からも呼べるよう、入れる間だけ割り込みを止める
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t next = (s_ev_head + 1u) % EVENT_FIFO_SIZE;
    if (next == s_ev_tail) {
        s_ev_dropped++;
    } else {
        float *ev = s_ev_fifo[s_ev_head];
        ev[0] = (float)code;
        ev[1] = a;
        ev[2] = b;
        ev[3] = c;
        ev[4] = d;
        ev[5] = e;
        s_ev_head = next;
    }
    __set_PRIMASK(primask);
}

// ISR: 記録する行に、順番待ちのイベントを1つ入れる(なければ 0)
static void TakeEvent(void) {
    if (s_ev_tail != s_ev_head) {
        memcpy(s_ev_row, s_ev_fifo[s_ev_tail], sizeof(s_ev_row));
        s_ev_tail = (s_ev_tail + 1u) % EVENT_FIFO_SIZE;
    } else {
        memset(s_ev_row, 0, sizeof(s_ev_row));
    }
}

uint32_t Logger_EventDropped(void) {
    return s_ev_dropped;
}

uint32_t Logger_EventQueued(void) {
    uint32_t head = s_ev_head;
    uint32_t tail = s_ev_tail;
    return (head + EVENT_FIFO_SIZE - tail) % EVENT_FIFO_SIZE;
}

uint32_t Logger_EventCapacity(void) {
    return EVENT_FIFO_SIZE - 1u; // 1つは空けておく(満杯と空を見分けるため)
}

void Logger_SetDirName(const char *name) {
    s_dir_name = name;
}

void Logger_SetFileName(const char *name) {
    s_file_name = name;
}

void Logger_SetIncludeTimestamp(bool enable) {
    s_include_timestamp = enable;
}

void Logger_SetDecimation(uint32_t every_n_ticks) {
    s_decimation = (every_n_ticks == 0) ? 1 : every_n_ticks;
    s_duration_ms = 0;
}

void Logger_SetDuration(uint32_t duration_ms) {
    s_duration_ms = duration_ms;
}

uint32_t Logger_RecordableMs(void) {
    return s_max_samples * s_decimation * LOGGER_TICK_MS;
}

void Logger_Start(void) {
    if (s_duration_ms > 0 && !s_stream) {
        uint32_t ticks = (s_duration_ms + LOGGER_TICK_MS - 1) / LOGGER_TICK_MS;
        s_decimation = (ticks + s_max_samples - 1) / s_max_samples; // 切り上げ
        if (s_decimation == 0) s_decimation = 1;
    }
    s_sample_count = 0;
    s_tick_count = 0;
    s_full = false;
    s_ev_head = 0;
    s_ev_tail = 0;
    s_ev_dropped = 0;
    if (s_stream) {
        s_blk_active = 0;
        s_blk_rows[0] = 0;
        s_blk_rows[1] = 0;
        s_blk_full[0] = false;
        s_blk_full[1] = false;
        s_stream_dropped = 0;
    }
    s_state = LOGGER_RECORDING; // 最後に立てる(ここからISRが記録を始める)
}

void Logger_Stop(void) {
    if (s_state == LOGGER_RECORDING) {
        s_state = LOGGER_STOPPED;
    }
}

// 流す方式の1行(ISR)
static void SampleStream(void) {
    uint8_t a = s_blk_active;
    if (s_blk_full[a]) {
        uint8_t o = (uint8_t)(a ^ 1u);
        if (s_blk_full[o]) { // 両方いっぱい(SD が追いついていない)
            s_stream_dropped++;
            return;
        }
        s_blk_rows[o] = 0;
        s_blk_active = o;
        a = o;
    }
    float *row = BlockBase(a) + STREAM_BLOCK_HEAD + s_blk_rows[a] * s_field_count;
    for (uint32_t i = 0; i < s_field_count; i++) {
        row[i] = *s_fields[i].value;
    }
    if (++s_blk_rows[a] >= s_blk_max_rows) s_blk_full[a] = true;
}

void Logger_Sample(void) {
    if (s_state != LOGGER_RECORDING) return;

    uint32_t tick = s_tick_count++;
    // 間引き: s_decimation tickに1回だけ記録する(開始直後のtickは記録する)
    if (tick % s_decimation != 0) return;

    s_time_s = (float)tick * CONTROL_DT_S;
    TakeEvent();

    if (s_stream) {
        SampleStream();
        return;
    }

    if (s_sample_count >= s_max_samples) {
        s_full = true;
        s_state = LOGGER_STOPPED;
        return;
    }

    float *row = &s_buffer[s_sample_count * s_field_count];
    for (uint32_t i = 0; i < s_field_count; i++) {
        row[i] = *s_fields[i].value;
    }
    s_sample_count++;
}

void Logger_Dump(void) {
    if (s_state != LOGGER_STOPPED || s_stream) {
        printf("no data to send!\r\n");
        return;
    }

    uint32_t size = s_sample_count * s_field_count * (uint32_t)sizeof(float);

    printf("BIN_START\r\n");
    printf("%s\r\n", s_dir_name);
    printf("%s\r\n", (s_file_name != NULL) ? s_file_name : "");
    printf("TIMESTAMP:%d\r\n", s_include_timestamp ? 1 : 0);
    printf("SIZE:%lu\r\n", (unsigned long)size);
    for (uint32_t i = 0; i < s_field_count; i++) {
        if (i > 0) printf(",");
        printf("%s", s_fields[i].name);
    }
    printf("\r\n");

    UART_WriteBytes((const uint8_t *)s_buffer, size);

    printf("BIN_END\r\n");
}

// ---- SD のファイル(すべてバイナリ。PC の tools/get_log.py が CSV に直す) ----
// 先頭は LOG_BIN_HEADER_BYTES(512)バイトちょうどで、最後の行の後ろを空白で埋める。
// 続くデータの書き始めがセクタの区切りにそろい、DMA で複数セクタをまとめて書けるようにするため
// (4バイト境界にそろっていないと、1セクタずつコピーして書く遅い方法になる)。
//
// YLOG1(RAM に貯めた記録を1つのファイルにする。Logger_SaveFile):
//   "YLOG1\n" "<列数>\n" "<列名0>,<列名1>,...<空白>\n" + float32 × 列数 × 行数
// YLOG2(走りながら流す。Logger_StreamBegin 〜 Logger_StreamEnd):
//   "YLOG1" の代わりに "YLOG2"、列名の行の後に "<ブロックのバイト数><空白>\n"
//   + ブロック × 個数。ブロックは [uint32 magic "YBLK", 通し番号, 行数, 列数] + float32 × 列数 × 行数
//   + 残りは使わない(ブロックの大きさは一定)。最後のブロックは途中までのことがある。
#define LOG_BIN_HEADER_BYTES 512u

// 先頭を組み立てる作業用バッファ(DMA で書くので 4 バイト境界にそろえる)
#define HEADER_BUF_SIZE 1024u
static char s_header_buf[HEADER_BUF_SIZE] __attribute__((aligned(4)));

// 先頭を s_header_buf に組み立てて、長さを返す(0 なら失敗)
static uint32_t BuildBinHeader(const char *magic, uint32_t block_bytes) {
    uint32_t n = (uint32_t)snprintf(s_header_buf, HEADER_BUF_SIZE, "%s\n%lu\n",
                                    magic, (unsigned long)s_field_count);
    for (uint32_t i = 0; i < s_field_count; i++) {
        n += (uint32_t)snprintf(&s_header_buf[n], HEADER_BUF_SIZE - n, "%s%s",
                                (i > 0) ? "," : "", s_fields[i].name);
    }
    if (block_bytes > 0) {
        n += (uint32_t)snprintf(&s_header_buf[n], HEADER_BUF_SIZE - n, "\n%lu", (unsigned long)block_bytes);
    }
    // 改行を含めて LOG_BIN_HEADER_BYTES(はみ出すなら 4 の倍数)になるよう、最後の改行の前を空白で埋める
    uint32_t total = (n + 1u <= LOG_BIN_HEADER_BYTES) ? LOG_BIN_HEADER_BYTES : ((n + 1u + 3u) & ~3u);
    if (total > HEADER_BUF_SIZE) return 0;
    while (n < total - 1u) s_header_buf[n++] = ' ';
    s_header_buf[n++] = '\n';
    return n;
}

static const char *FileNameOrDefault(void) {
    return (s_file_name != NULL && s_file_name[0] != '\0') ? s_file_name : "log";
}

bool Logger_SaveFile(char *path_out, uint32_t path_len) {
    if (s_state != LOGGER_STOPPED || s_stream) {
        printf("no data to save!\r\n");
        return false;
    }
    if (!SDCard_IsMounted()) return false;
    if (!SDCard_OpenNewSequential(s_dir_name, FileNameOrDefault(), "bin", path_out, path_len)) return false;

    uint32_t n = BuildBinHeader("YLOG1", 0);
    bool ok = (n > 0) && SDCard_Write(s_header_buf, n);
    uint32_t rows_bytes = s_sample_count * s_field_count * (uint32_t)sizeof(float);
    if (ok && rows_bytes > 0) ok = SDCard_Write(s_buffer, rows_bytes);

    // 書き込みに失敗しても閉じる(開いたままだと次が開けない)
    bool closed = SDCard_Close();
    return ok && closed;
}

// ---- 流す方式 ----

bool Logger_StreamBegin(char *path_out, uint32_t path_len) {
    if (s_state == LOGGER_RECORDING || s_stream) return false;
    if (!SDCard_IsMounted()) return false;

    uint32_t n = BuildBinHeader("YLOG2", STREAM_BLOCK_BYTES);
    if (n != LOG_BIN_HEADER_BYTES) return false; // 列名が長すぎて 512 に収まらない
    if (!SDCard_StreamOpen(s_dir_name, FileNameOrDefault(), "bin", LOGGER_STREAM_RESERVE_BYTES,
                           path_out, path_len)) {
        return false;
    }
    // 先頭を書く(走る前なので、書き終わるまで待つ)
    bool ok = SDCard_StreamWrite(0, s_header_buf, LOG_BIN_HEADER_BYTES);
    uint32_t t0 = HAL_GetTick();
    int r = 1;
    while (ok && (r = SDCard_StreamPoll()) == 1) {
        if (HAL_GetTick() - t0 > 1000u) {
            ok = false;
            break;
        }
    }
    if (!ok || r != 0) {
        SDCard_StreamClose(LOG_BIN_HEADER_BYTES);
        return false;
    }

    s_blk_max_rows = (STREAM_BLOCK_FLOATS - STREAM_BLOCK_HEAD) / s_field_count;
    s_next_write = 0;
    s_writing = -1;
    s_blk_seq = 0;
    s_stream_failed = false;
    s_stream_dropped = 0;
    s_write_max_ms = 0;
    s_write_total_ms = 0;
    s_stream = true;
    return true;
}

void Logger_StreamGetStats(LoggerStreamStats *st) {
    st->blocks = s_blk_seq;
    st->dropped_rows = s_stream_dropped;
    st->write_max_ms = s_write_max_ms;
    st->write_avg_ms = (s_blk_seq > 0) ? s_write_total_ms / s_blk_seq : 0;
    st->rows_per_block = s_blk_max_rows;
}

void Logger_StreamPoll(void) {
    if (!s_stream) return;

    // 送っている最中: 終わったかを見る(カードへの問い合わせは 1ms に1回まで)
    if (s_writing >= 0) {
        uint32_t now = HAL_GetTick();
        if (now == s_last_poll_ms) return;
        s_last_poll_ms = now;
        int r = SDCard_StreamPoll();
        if (r == 1) return;
        if (r < 0) {
            s_stream_failed = true;
            s_writing = -1;
            return;
        }
        s_blk_full[s_writing] = false; // このブロックはまた ISR が使える
        uint32_t took = now - s_write_start_ms;
        if (took > s_write_max_ms) s_write_max_ms = took;
        s_write_total_ms += took;
        s_blk_seq++;
        s_next_write = (uint8_t)(s_writing ^ 1);
        s_writing = -1;
    }
    if (s_stream_failed) return;

    // 次のブロックがいっぱいなら送り出す
    uint8_t b = s_next_write;
    if (!s_blk_full[b]) return;
    uint32_t offset = LOG_BIN_HEADER_BYTES + s_blk_seq * STREAM_BLOCK_BYTES;
    if (offset + STREAM_BLOCK_BYTES > LOGGER_STREAM_RESERVE_BYTES) { // 確保した領域を使い切った
        s_stream_failed = true;
        return;
    }
    uint32_t *head = (uint32_t *)BlockBase(b);
    head[0] = STREAM_BLOCK_MAGIC;
    head[1] = s_blk_seq;
    head[2] = s_blk_rows[b];
    head[3] = s_field_count;
    if (SDCard_StreamWrite(offset, head, STREAM_BLOCK_BYTES)) {
        s_writing = (int8_t)b;
        s_write_start_ms = HAL_GetTick();
    }
}

bool Logger_StreamEnd(uint32_t *dropped_rows) {
    if (!s_stream) return false;
    s_state = LOGGER_STOPPED; // ここから ISR は書かない

    // 途中までのブロックも送る
    uint8_t a = s_blk_active;
    if (!s_blk_full[a] && s_blk_rows[a] > 0) s_blk_full[a] = true;

    uint32_t t0 = HAL_GetTick();
    while (!s_stream_failed && (s_writing >= 0 || s_blk_full[0] || s_blk_full[1])) {
        Logger_StreamPoll();
        if (HAL_GetTick() - t0 > 3000u) {
            s_stream_failed = true;
            break;
        }
    }
    bool closed = SDCard_StreamClose(LOG_BIN_HEADER_BYTES + s_blk_seq * STREAM_BLOCK_BYTES);
    if (dropped_rows != NULL) *dropped_rows = s_stream_dropped;
    s_stream = false;
    return !s_stream_failed && closed;
}

bool Logger_StreamFailed(void) {
    return s_stream && s_stream_failed;
}

bool Logger_IsRecording(void) {
    return s_state == LOGGER_RECORDING;
}

bool Logger_IsFull(void) {
    return s_full;
}

uint32_t Logger_SampleCount(void) {
    return s_sample_count;
}

uint32_t Logger_FieldCount(void) {
    return s_field_count;
}
