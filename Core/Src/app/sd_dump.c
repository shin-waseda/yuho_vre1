#include "app/sd_dump.h"

#include <stdio.h>
#include <string.h>
#include "interface/sdcard.h"
#include "interface/uart.h"
#include "interface/led.h"
#include "app/mode_ui.h"

// 1回の一覧で集めるファイル数。これより多ければページをめくるように続きを読む。
#define SD_DUMP_LIST_MAX 16
#define SD_DUMP_CHUNK    512

static char s_paths[SD_DUMP_LIST_MAX][SDCARD_PATH_MAX];
static uint8_t s_chunk[SD_DUMP_CHUNK] __attribute__((aligned(4)));

// 1ファイルを枠付きで送る。PCへは label をパスとして伝える。
// 途中で読めなくなっても、PC側の受信がずれないよう SIZEぶんは必ず送り切り
// (残りは0)、最後をFILE_ERRORにする。
static bool SendFile(const char *path, const char *label) {
    uint32_t size = 0;
    if (!SDCard_OpenRead(path, &size)) return false;

    printf("FILE_START\r\n%s\r\nSIZE:%lu\r\n", label, (unsigned long)size);

    bool ok = true;
    uint32_t left = size;
    while (left > 0) {
        uint32_t want = (left < SD_DUMP_CHUNK) ? left : SD_DUMP_CHUNK;
        uint32_t got = 0;
        if (ok) {
            ok = SDCard_Read(s_chunk, want, &got) && (got == want);
        }
        if (!ok) {
            memset(s_chunk, 0, want);
        }
        UART_WriteBytes(s_chunk, want);
        left -= want;
    }
    SDCard_Close();

    printf(ok ? "FILE_END\r\n" : "FILE_ERROR\r\n");
    return ok;
}

// "sent/straight/x.csv" → "straight/x.csv"(PCでは元の場所に置かせる)
static const char *OriginalPath(const char *path) {
    size_t n = strlen(SDCARD_SENT_DIR);
    if (strncmp(path, SDCARD_SENT_DIR, n) == 0 && path[n] == '/') return path + n + 1;
    return path;
}

// 未送信をすべて送り、送ったものは sent/ へ移す。送った数を返す(失敗したらそこで止める)。
static int SendUnsentAndMove(bool *failed) {
    int sent = 0;
    *failed = false;
    while (1) {
        int n = SDCard_ListUnsent(s_paths, SD_DUMP_LIST_MAX);
        if (n < 0) {
            *failed = true;
            break;
        }
        if (n == 0) break;

        for (int i = 0; i < n; i++) {
            // 送れなかった・移動できなかったファイルは次の一覧にまた出てくるので、
            // 同じものを送り続けないよう、失敗したらそこで打ち切る。
            if (!SendFile(s_paths[i], s_paths[i]) || !SDCard_MoveToSent(s_paths[i])) {
                *failed = true;
                return sent;
            }
            sent++;
        }
    }
    return sent;
}

// base配下のCSVを、移動せずにすべて送る。送った数を返す。
// 移動しないので、一覧はskipで読み進める(16個ずつ)。失敗したファイルは飛ばして続ける。
static int SendAllUnder(const char *base, bool *failed) {
    int sent = 0;
    int skip = 0;
    while (1) {
        int n = SDCard_ListCsv(base, skip, s_paths, SD_DUMP_LIST_MAX);
        if (n < 0) {
            *failed = true;
            break;
        }
        for (int i = 0; i < n; i++) {
            if (SendFile(s_paths[i], OriginalPath(s_paths[i]))) {
                sent++;
            } else {
                *failed = true;
            }
        }
        if (n < SD_DUMP_LIST_MAX) break; // 最後まで読んだ
        skip += n;
    }
    return sent;
}

static void WaitForeverIfNotMounted(const char *name) {
    if (SDCard_IsMounted()) return;
    printf("%s: SD card is not mounted\r\n", name);
    LED_SetShiftPattern(0x0055u); // LED1,3,5,7 = 失敗
    while (1) {
        ModeUI_WaitClick(); // フェイルセーフの監視だけ続ける
    }
}

void SdDump_Run(void) {
    WaitForeverIfNotMounted("SD DUMP");

    while (1) {
        printf("SD DUMP: press button to send unsent logs\r\n");
        ModeUI_WaitClick();

        bool failed = false;
        int sent = SendUnsentAndMove(&failed);
        printf("SD DUMP: sent %d file(s)%s\r\n", sent, failed ? ", stopped by an error" : "");
        LED_SetShiftPattern(failed ? 0x0055u : 0x007Fu);
    }
}

void SdDumpAll_Run(void) {
    WaitForeverIfNotMounted("SD DUMP ALL");

    while (1) {
        printf("SD DUMP ALL: press button to send ALL logs (including sent/)\r\n");
        ModeUI_WaitClick();

        bool failed = false;
        int sent = SendAllUnder("", &failed);               // 未送信
        sent += SendAllUnder(SDCARD_SENT_DIR, &failed);     // 送信済み
        printf("SD DUMP ALL: sent %d file(s)%s\r\n", sent, failed ? ", some failed" : "");
        LED_SetShiftPattern(failed ? 0x0055u : 0x007Fu);
    }
}
