#include "main.h"
#include "sdio.h"   // MX_SDIO_SD_Init
#include "fatfs.h"  // SDFatFS / SDPath (MX_FATFS_Init()でドライバ登録済み)
#include "bsp_driver_sd.h"
#include "interface/sdcard.h"

#include <stdio.h>
#include <string.h>

static bool s_mounted = false;
static bool s_file_open = false;
static FIL s_file;
static uint32_t s_written_total = 0; // 今開いているファイルに書けた合計(診断用)

// 診断用: 失敗した直後に、HALが記録したSDIOのエラーとカードの状態を表示する。
// ErrorCodeはHALの各操作の開始時にクリアされるので、直前の操作の結果になる。
static void PrintHalDiag(void) {
    uint32_t err = HAL_SD_GetError(&hsd);
    printf("  HAL: ErrorCode=0x%08lX state=%d", (unsigned long)err, (int)HAL_SD_GetState(&hsd));
    if (err & HAL_SD_ERROR_CMD_CRC_FAIL)     printf(" CMD_CRC_FAIL");
    if (err & HAL_SD_ERROR_DATA_CRC_FAIL)    printf(" DATA_CRC_FAIL");
    if (err & HAL_SD_ERROR_CMD_RSP_TIMEOUT)  printf(" CMD_RSP_TIMEOUT");
    if (err & HAL_SD_ERROR_DATA_TIMEOUT)     printf(" DATA_TIMEOUT");
    if (err & HAL_SD_ERROR_TX_UNDERRUN)      printf(" TX_UNDERRUN");
    if (err & HAL_SD_ERROR_RX_OVERRUN)       printf(" RX_OVERRUN");
    if (err & HAL_SD_ERROR_DMA)              printf(" DMA");
    if (err & HAL_SD_ERROR_TIMEOUT)          printf(" TIMEOUT");
    printf("\r\n");

    // カードの状態はCMD13で問い合わせる(4=TRANSFER(待機), 6=RECEIVING, 7=PROGRAMMING(書込中))
    HAL_SD_CardStateTypeDef cs = HAL_SD_GetCardState(&hsd);
    printf("  card state=%lu (4=TRANSFER 6=RECEIVING 7=PROGRAMMING)\r\n", (unsigned long)cs);
}

// ============================================================
// FATFS/Target/bsp_driver_sd.c の __weak 関数の上書き
// (生成ファイルを触らずに直すため、ここで同名の関数を定義する)
//
// 生成された sd_diskio.c(DMAテンプレート)の問題:
//  - SD_status() は、その瞬間にカードがTRANSFER(待機)でなければ STA_NOINIT を返す。
//    FatFsは f_write() 等の最初に disk_status() で検査するので、カードが前の書き込みを
//    内部処理中(PROGRAMMING, 数ms)だと FR_INVALID_OBJECT(9) で失敗する。
//  - SD_write() の scratch buffer 経路(4バイト境界にないバッファ)は、書き終えた後に
//    カードがTRANSFERに戻るのを待たずに戻るため、上の状態が実際に起きる。
// 対策: カードの状態を問い合わせる時・転送を始める前に、TRANSFERに戻るまで待つ。
// ============================================================

#define SD_READY_TIMEOUT_MS 500

// カードがTRANSFER(待機)に戻るまで待つ。タイムアウトならfalse。
static bool WaitCardReady(uint32_t timeout_ms) {
    uint32_t t0 = HAL_GetTick();
    while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER) {
        if (HAL_GetTick() - t0 >= timeout_ms) return false;
    }
    return true;
}

uint8_t BSP_SD_GetCardState(void) {
    return WaitCardReady(SD_READY_TIMEOUT_MS) ? SD_TRANSFER_OK : SD_TRANSFER_BUSY;
}

uint8_t BSP_SD_ReadBlocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks) {
    if (!WaitCardReady(SD_READY_TIMEOUT_MS)) return MSD_ERROR;
    if (HAL_SD_ReadBlocks_DMA(&hsd, (uint8_t *)pData, ReadAddr, NumOfBlocks) != HAL_OK) {
        return MSD_ERROR;
    }
    return MSD_OK;
}

uint8_t BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks) {
    if (!WaitCardReady(SD_READY_TIMEOUT_MS)) return MSD_ERROR;
    if (HAL_SD_WriteBlocks_DMA(&hsd, (uint8_t *)pData, WriteAddr, NumOfBlocks) != HAL_OK) {
        return MSD_ERROR;
    }
    return MSD_OK;
}

// ============================================================

bool SDCard_Mount(void) {
    // .iocで「Do Not Generate Function Call」にしているので、hsdの設定(Instance,
    // ClockDiv等)はここで行う。HAL_SD_Init()はf_mount内のBSP_SD_Init()が呼ぶ。
    MX_SDIO_SD_Init();

    // opt=1: 今すぐマウントする(カードの初期化まで行い、結果を返す)
    FRESULT res = f_mount(&SDFatFS, SDPath, 1);
    s_mounted = (res == FR_OK);
    if (!s_mounted) {
        printf("SD: mount failed (FRESULT=%d)\r\n", (int)res);
        PrintHalDiag();
    }
    return s_mounted;
}

bool SDCard_IsMounted(void) {
    return s_mounted;
}

// dirがなければ作る(既にあればOK)
static bool EnsureDir(const char *dir) {
    FRESULT res = f_mkdir(dir);
    if (res != FR_OK && res != FR_EXIST) {
        printf("SD: mkdir %s failed (FRESULT=%d)\r\n", dir, (int)res);
        PrintHalDiag();
        return false;
    }
    return true;
}

static bool Exists(const char *path) {
    FILINFO fno;
    return f_stat(path, &fno) == FR_OK;
}

// dir/prefix_NNNN の番号が、ログの拡張子(.csv / .bin)のどれかで、dir か sent/dir に使われているか
static bool NumberUsed(const char *dir, const char *prefix, uint32_t i) {
    static const char *const kExts[] = { "csv", "bin" };
    char path[SDCARD_PATH_MAX];
    char sent_path[SDCARD_PATH_MAX + sizeof(SDCARD_SENT_DIR)];
    for (uint32_t e = 0; e < sizeof(kExts) / sizeof(kExts[0]); e++) {
        snprintf(path, sizeof(path), "%s/%s_%04lu.%s", dir, prefix, (unsigned long)i, kExts[e]);
        snprintf(sent_path, sizeof(sent_path), "%s/%s", SDCARD_SENT_DIR, path);
        if (Exists(path) || Exists(sent_path)) return true;
    }
    return false;
}

bool SDCard_OpenNewSequential(const char *dir, const char *prefix, const char *ext,
                              char *path_out, uint32_t path_len) {
    if (!s_mounted || s_file_open) return false;
    if (!EnsureDir(dir)) return false;

    char path[SDCARD_PATH_MAX];
    // 1から順に、まだ使っていない番号を探す。送信済み(sent/へ移動済み)の番号も
    // 使用済みとみなす(でないと送信後に同じ名前が再び作られ、移動・PC保存で衝突する)。
    // 拡張子が違っても(.csv と .bin)同じ番号は使わない(PC で .bin から .csv を作ったときに、
    // 古い同じ番号の .csv とぶつからないように)。
    for (uint32_t i = 1; i <= 9999; i++) {
        if (NumberUsed(dir, prefix, i)) continue;
        snprintf(path, sizeof(path), "%s/%s_%04lu.%s", dir, prefix, (unsigned long)i, ext);

        FRESULT res = f_open(&s_file, path, FA_CREATE_NEW | FA_WRITE);
        if (res == FR_EXIST) continue;
        if (res != FR_OK) {
            printf("SD: open %s failed (FRESULT=%d)\r\n", path, (int)res);
            PrintHalDiag();
            return false;
        }
        s_file_open = true;
        s_written_total = 0;
        if (path_out != NULL && path_len > 0) {
            snprintf(path_out, path_len, "%s", path);
        }
        return true;
    }
    printf("SD: no free number in %s/%s_####.%s\r\n", dir, prefix, ext);
    return false;
}

bool SDCard_Write(const void *data, uint32_t len) {
    if (!s_file_open) return false;
    UINT written = 0;
    FRESULT res = f_write(&s_file, data, len, &written);
    if (res != FR_OK || written != len) {
        printf("SD: write failed (FRESULT=%d, %lu/%lu, file total before this write=%lu)\r\n",
               (int)res, (unsigned long)written, (unsigned long)len,
               (unsigned long)s_written_total);
        PrintHalDiag();
        return false;
    }
    s_written_total += written;
    return true;
}

bool SDCard_OpenRead(const char *path, uint32_t *size_out) {
    if (!s_mounted || s_file_open) return false;
    FRESULT res = f_open(&s_file, path, FA_OPEN_EXISTING | FA_READ);
    if (res != FR_OK) {
        printf("SD: open %s failed (FRESULT=%d)\r\n", path, (int)res);
        return false;
    }
    s_file_open = true;
    if (size_out != NULL) *size_out = (uint32_t)f_size(&s_file);
    return true;
}

bool SDCard_Read(void *buf, uint32_t len, uint32_t *got_out) {
    if (!s_file_open) return false;
    UINT got = 0;
    FRESULT res = f_read(&s_file, buf, len, &got);
    if (got_out != NULL) *got_out = (uint32_t)got;
    if (res != FR_OK) {
        printf("SD: read failed (FRESULT=%d)\r\n", (int)res);
        PrintHalDiag();
        return false;
    }
    return true;
}

bool SDCard_Sync(void) {
    if (!s_file_open) return false;
    FRESULT res = f_sync(&s_file);
    if (res != FR_OK) {
        printf("SD: sync failed (FRESULT=%d)\r\n", (int)res);
        PrintHalDiag();
        return false;
    }
    return true;
}

// ============================================================
// 待たない書き込み(走りながらログを流すため)
// ファイルを先に reserve バイトまで伸ばして領域を確保し、高速シークのクラスタの対応表(cltbl)から
// ファイルの中の位置 → カードのセクタ番号を自分で計算して、HAL_SD_WriteBlocks_DMA を始めるだけで戻る。
// FatFs を通さないので、f_write のように DMA の終わりやカードの書き込み(PROGRAMMING)を待たない。
// 終わったかは SDCard_StreamPoll() で見に行く。確保した領域が途中で途切れていても、
// 途切れ目で DMA を分けて書く。
// ============================================================

#define STREAM_CLMT_ITEMS 64 // クラスタの対応表の大きさ(途切れ 31 か所まで)

typedef enum {
    STREAM_IDLE,     // 書いていない
    STREAM_WAIT,     // カードが待機(TRANSFER)に戻るのを待ってから、次の DMA を始める
    STREAM_DMA,      // DMA で送っている
    STREAM_ERROR,
} StreamState;

static DWORD s_clmt[STREAM_CLMT_ITEMS];
static bool s_stream_open = false;
static uint32_t s_stream_reserve = 0;
static StreamState s_stream_state = STREAM_IDLE;
static const uint8_t *s_stream_buf = NULL; // これから送るデータ
static uint32_t s_stream_off = 0;          // そのファイルの中の位置[バイト]
static uint32_t s_stream_left = 0;         // 残り[バイト]
static uint32_t s_stream_run = 0;          // 今の DMA で送っているセクタ数

// ファイルの中の位置 off(セクタ境界)の、カードのセクタ番号と、そこから連続しているセクタ数
static bool OffsetToSector(uint32_t off, uint32_t *sector, uint32_t *contig) {
    FATFS *fs = s_file.obj.fs;
    uint32_t clsz = (uint32_t)fs->csize * 512u;
    uint32_t cl = off / clsz;            // 何番目のクラスタか
    uint32_t in_cl = (off % clsz) / 512u; // クラスタの中の何番目のセクタか
    const DWORD *t = &s_clmt[1];         // (クラスタ数, 始まりのクラスタ) の組の並び。0 で終わり
    while (t[0] != 0) {
        if (cl < t[0]) {
            uint32_t clst = t[1] + cl;
            *sector = fs->database + (clst - 2u) * fs->csize + in_cl;
            *contig = (t[0] - cl) * fs->csize - in_cl;
            return true;
        }
        cl -= t[0];
        t += 2;
    }
    return false;
}

// 残りのうち、連続している所まで DMA を始める
static void StreamStartRun(void) {
    uint32_t sector, contig;
    if (!OffsetToSector(s_stream_off, &sector, &contig)) {
        s_stream_state = STREAM_ERROR;
        return;
    }
    uint32_t n = s_stream_left / 512u;
    if (n > contig) n = contig;
    if (HAL_SD_WriteBlocks_DMA(&hsd, (uint8_t *)s_stream_buf, sector, n) != HAL_OK) {
        printf("SD: stream DMA start failed\r\n");
        PrintHalDiag();
        s_stream_state = STREAM_ERROR;
        return;
    }
    s_stream_run = n;
    s_stream_state = STREAM_DMA;
}

bool SDCard_StreamOpen(const char *dir, const char *prefix, const char *ext, uint32_t reserve,
                       char *path_out, uint32_t path_len) {
    if (s_stream_open) return false;
    if (!SDCard_OpenNewSequential(dir, prefix, ext, path_out, path_len)) return false;

    // 先に伸ばして領域を確保する(書き込みで開いたファイルは、終わりより先へ f_lseek すると伸びる)
    FRESULT res = f_lseek(&s_file, reserve);
    if (res != FR_OK || f_size(&s_file) != reserve || f_sync(&s_file) != FR_OK) {
        printf("SD: stream reserve failed (FRESULT=%d)\r\n", (int)res);
        SDCard_Close();
        return false;
    }
    // クラスタの対応表を作る
    s_clmt[0] = STREAM_CLMT_ITEMS;
    s_file.cltbl = s_clmt;
    res = f_lseek(&s_file, CREATE_LINKMAP);
    if (res != FR_OK) {
        printf("SD: stream link map failed (FRESULT=%d, file too fragmented?)\r\n", (int)res);
        s_file.cltbl = NULL;
        SDCard_Close();
        return false;
    }
    s_stream_open = true;
    s_stream_reserve = reserve;
    s_stream_state = STREAM_IDLE;
    return true;
}

bool SDCard_StreamWrite(uint32_t offset, const void *data, uint32_t len) {
    if (!s_stream_open || s_stream_state != STREAM_IDLE) return false;
    if ((offset % 512u) != 0 || (len % 512u) != 0 || len == 0) return false;
    if (offset + len > s_stream_reserve) return false;
    s_stream_buf = (const uint8_t *)data;
    s_stream_off = offset;
    s_stream_left = len;
    s_stream_state = STREAM_WAIT; // カードが待機に戻ったら始める(次の Poll で)
    return true;
}

int SDCard_StreamPoll(void) {
    switch (s_stream_state) {
        case STREAM_DMA:
            if (HAL_SD_GetState(&hsd) != HAL_SD_STATE_READY) return 1; // まだ送っている
            if (HAL_SD_GetError(&hsd) != HAL_SD_ERROR_NONE) {
                printf("SD: stream DMA failed\r\n");
                PrintHalDiag();
                s_stream_state = STREAM_ERROR;
                return -1;
            }
            s_stream_buf += s_stream_run * 512u;
            s_stream_off += s_stream_run * 512u;
            s_stream_left -= s_stream_run * 512u;
            s_stream_state = STREAM_WAIT; // カードが書き終わる(PROGRAMMING → TRANSFER)のを待つ
            return 1;
        case STREAM_WAIT:
            if (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER) return 1; // カードが書いている
            if (s_stream_left == 0) {
                s_stream_state = STREAM_IDLE;
                return 0;
            }
            StreamStartRun();
            return (s_stream_state == STREAM_ERROR) ? -1 : 1;
        case STREAM_IDLE:
            return 0;
        case STREAM_ERROR:
        default:
            return -1;
    }
}

bool SDCard_StreamClose(uint32_t final_size) {
    if (!s_stream_open) return false;
    // 書き終わるまで待つ(止まっているときに呼ぶ想定)
    uint32_t t0 = HAL_GetTick();
    while (SDCard_StreamPoll() == 1) {
        if (HAL_GetTick() - t0 > 2000u) {
            s_stream_state = STREAM_ERROR;
            break;
        }
    }
    bool ok = (s_stream_state == STREAM_IDLE);
    s_stream_open = false;

    // 書いた所までに縮めて閉じる(高速シークの表は外してから)
    s_file.cltbl = NULL;
    if (final_size > s_stream_reserve) final_size = s_stream_reserve;
    if (f_lseek(&s_file, final_size) != FR_OK || f_truncate(&s_file) != FR_OK) ok = false;
    bool closed = SDCard_Close();
    return ok && closed;
}

bool SDCard_Close(void) {
    if (!s_file_open) return false;
    s_file_open = false;
    FRESULT res = f_close(&s_file);
    if (res != FR_OK) {
        printf("SD: close failed (FRESULT=%d)\r\n", (int)res);
        PrintHalDiag();
        return false;
    }
    return true;
}

// 拡張子(".xyz" の3文字)が ext と同じか(大文字・小文字は区別しない)
static bool HasExt(const char *name, const char *ext) {
    size_t n = strlen(name);
    if (n < 5) return false;
    const char *e = name + n - 4;
    if (e[0] != '.') return false;
    for (int i = 0; i < 3; i++) {
        char c = e[1 + i];
        if (c >= 'A' && c <= 'Z') c = (char)(c - 'A' + 'a');
        if (c != ext[i]) return false;
    }
    return true;
}

// 送る対象のログ(CSV と、探索などの追記用のバイナリ .bin)
static bool IsCsv(const char *name) {
    return HasExt(name, "csv") || HasExt(name, "bin");
}

// 一覧の対象外にするフォルダ(送信済み・隠し・Windowsが作るもの)
static bool IsSkippedDir(const char *name) {
    return strcmp(name, SDCARD_SENT_DIR) == 0 || name[0] == '.'
        || strcmp(name, "System Volume Information") == 0;
}

// 一覧の途中経過。skip個を読み飛ばしてから、最大max個をpathsへ入れる。
typedef struct {
    char (*paths)[SDCARD_PATH_MAX];
    int max;
    int skip;    // まだ読み飛ばす残り
    int n;       // 入れた数
} ListState;

// CSVを1つ見つけるたびに呼ぶ。長すぎるパスは扱わない(数にも入れない)。
static void AddPath(ListState *st, const char *base, const char *dir, const char *name) {
    char path[SDCARD_PATH_MAX];
    int len;
    if (base[0] == '\0' && dir[0] == '\0') {
        len = snprintf(path, sizeof(path), "%s", name);
    } else if (base[0] == '\0') {
        len = snprintf(path, sizeof(path), "%s/%s", dir, name);
    } else if (dir[0] == '\0') {
        len = snprintf(path, sizeof(path), "%s/%s", base, name);
    } else {
        len = snprintf(path, sizeof(path), "%s/%s/%s", base, dir, name);
    }
    if (len < 0 || len >= SDCARD_PATH_MAX) return;

    if (st->skip > 0) {
        st->skip--;
        return;
    }
    snprintf(st->paths[st->n], SDCARD_PATH_MAX, "%s", path);
    st->n++;
}

int SDCard_ListCsv(const char *base, int skip, char (*paths)[SDCARD_PATH_MAX], int max) {
    if (!s_mounted) return -1;

    // base直下と、その1段下のフォルダのCSVを集める(sent/・隠しフォルダ等は除く)。
    // 一覧を作る間はファイルを移動しない(読みながら動かすと読み飛ばすおそれがある)。
    // FILINFOはLFN用に1個約270byteあるので、スタックではなくstaticに置く
    static DIR root, sub;
    static FILINFO fno, sfno;
    FRESULT res = f_opendir(&root, base);
    if (res == FR_NO_PATH) return 0; // baseがまだない(sent/が未作成など)
    if (res != FR_OK) return -1;

    ListState st = { .paths = paths, .max = max, .skip = skip, .n = 0 };
    while (st.n < st.max) {
        if (f_readdir(&root, &fno) != FR_OK || fno.fname[0] == '\0') break;

        if (fno.fattrib & AM_DIR) {
            if (IsSkippedDir(fno.fname)) continue;

            char dir[SDCARD_PATH_MAX];
            char dir_path[SDCARD_PATH_MAX];
            snprintf(dir, sizeof(dir), "%s", fno.fname);
            if (base[0] == '\0') {
                snprintf(dir_path, sizeof(dir_path), "%s", dir);
            } else {
                snprintf(dir_path, sizeof(dir_path), "%s/%s", base, dir);
            }
            if (f_opendir(&sub, dir_path) != FR_OK) continue;
            while (st.n < st.max) {
                if (f_readdir(&sub, &sfno) != FR_OK || sfno.fname[0] == '\0') break;
                if (!(sfno.fattrib & AM_DIR) && IsCsv(sfno.fname)) {
                    AddPath(&st, base, dir, sfno.fname);
                }
            }
            f_closedir(&sub);
        } else if (IsCsv(fno.fname)) {
            AddPath(&st, base, "", fno.fname);
        }
    }
    f_closedir(&root);
    return st.n;
}

int SDCard_ListUnsent(char (*paths)[SDCARD_PATH_MAX], int max) {
    return SDCard_ListCsv("", 0, paths, max);
}

bool SDCard_MoveToSent(const char *path) {
    if (!s_mounted || s_file_open) return false;
    if (!EnsureDir(SDCARD_SENT_DIR)) return false;

    // path が "dir/name" なら sent/dir を作る
    const char *slash = strrchr(path, '/');
    if (slash != NULL) {
        char sent_dir[SDCARD_PATH_MAX + sizeof(SDCARD_SENT_DIR)];
        snprintf(sent_dir, sizeof(sent_dir), "%s/%.*s", SDCARD_SENT_DIR, (int)(slash - path), path);
        if (!EnsureDir(sent_dir)) return false;
    }

    char dst[SDCARD_PATH_MAX + sizeof(SDCARD_SENT_DIR)];
    snprintf(dst, sizeof(dst), "%s/%s", SDCARD_SENT_DIR, path);
    FRESULT res = f_rename(path, dst);
    if (res != FR_OK) {
        printf("SD: move %s -> %s failed (FRESULT=%d)\r\n", path, dst, (int)res);
        PrintHalDiag();
        return false;
    }
    return true;
}
