#include "app/logger.h"

#include <stdio.h>
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
static float s_buffer[LOGGER_BUFFER_SIZE];

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
    if (s_duration_ms > 0) {
        uint32_t ticks = (s_duration_ms + LOGGER_TICK_MS - 1) / LOGGER_TICK_MS;
        s_decimation = (ticks + s_max_samples - 1) / s_max_samples; // 切り上げ
        if (s_decimation == 0) s_decimation = 1;
    }
    s_sample_count = 0;
    s_tick_count = 0;
    s_full = false;
    s_state = LOGGER_RECORDING; // 最後に立てる(ここからISRが記録を始める)
}

void Logger_Stop(void) {
    if (s_state == LOGGER_RECORDING) {
        s_state = LOGGER_STOPPED;
    }
}

void Logger_Sample(void) {
    if (s_state != LOGGER_RECORDING) return;

    uint32_t tick = s_tick_count++;
    // 間引き: s_decimation tickに1回だけ記録する(開始直後のtickは記録する)
    if (tick % s_decimation != 0) return;

    if (s_sample_count >= s_max_samples) {
        s_full = true;
        s_state = LOGGER_STOPPED;
        return;
    }

    s_time_s = (float)tick * CONTROL_DT_S;

    float *row = &s_buffer[s_sample_count * s_field_count];
    for (uint32_t i = 0; i < s_field_count; i++) {
        row[i] = *s_fields[i].value;
    }
    s_sample_count++;
}

void Logger_Dump(void) {
    if (s_state != LOGGER_STOPPED) {
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

// CSVを溜めてからまとめてf_writeする作業用バッファ。
// 1行は最大でも 16列 × 16文字 程度なので、残りがこれを切ったら書き出す。
#define CSV_BUF_SIZE      1024
#define CSV_LINE_RESERVE  320
static char s_csv_buf[CSV_BUF_SIZE] __attribute__((aligned(4)));

bool Logger_SaveCSV(char *path_out, uint32_t path_len) {
    if (s_state != LOGGER_STOPPED) {
        printf("no data to save!\r\n");
        return false;
    }
    if (!SDCard_IsMounted()) return false;

    const char *file = (s_file_name != NULL && s_file_name[0] != '\0') ? s_file_name : "log";
    if (!SDCard_OpenNewSequential(s_dir_name, file, "csv", path_out, path_len)) return false;

    bool ok = true;
    uint32_t n = 0;

    // 1行目: 列名
    for (uint32_t i = 0; i < s_field_count; i++) {
        n += (uint32_t)snprintf(&s_csv_buf[n], CSV_BUF_SIZE - n, "%s%s",
                                (i > 0) ? "," : "", s_fields[i].name);
    }
    n += (uint32_t)snprintf(&s_csv_buf[n], CSV_BUF_SIZE - n, "\r\n");

    // 2行目以降: データ
    for (uint32_t r = 0; r < s_sample_count && ok; r++) {
        const float *row = &s_buffer[r * s_field_count];
        for (uint32_t i = 0; i < s_field_count; i++) {
            n += (uint32_t)snprintf(&s_csv_buf[n], CSV_BUF_SIZE - n, "%s%.6g",
                                    (i > 0) ? "," : "", (double)row[i]);
        }
        n += (uint32_t)snprintf(&s_csv_buf[n], CSV_BUF_SIZE - n, "\r\n");

        if (CSV_BUF_SIZE - n < CSV_LINE_RESERVE) {
            ok = SDCard_Write(s_csv_buf, n);
            n = 0;
        }
    }
    if (ok && n > 0) {
        ok = SDCard_Write(s_csv_buf, n);
    }

    // 書き込みに失敗しても閉じる(開いたままだと次が開けない)
    bool closed = SDCard_Close();
    return ok && closed;
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
