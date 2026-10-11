#include "plant_bridge.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#ifdef _WIN32
#include <direct.h>
#endif

#include "params.h"

#define PATH_MAX_LEN 1024
#define CMD_MAX_LEN 4096

static const float kSearchSpeeds[] = SPEED_SELECT_SEARCH_V_MM_S;
static const float kTurnSpeeds[] = SPEED_SELECT_SEARCH_TURN_V_MM_S;
static const float kAccels[] = SPEED_SELECT_ACCEL_MM_S2;

#define COUNT_OF(a) ((int)(sizeof(a) / sizeof((a)[0])))

// 表の中の番号(なければ −1)
static int IndexOf(const float *values, int count, float v) {
    for (int i = 0; i < count; i++) {
        if (fabsf(values[i] - v) < 0.5f) return i;
    }
    return -1;
}

static void PrintChoices(const char *name, const float *values, int count) {
    fprintf(stderr, "  %s:", name);
    for (int i = 0; i < count; i++) fprintf(stderr, " %.0f", (double)values[i]);
    fprintf(stderr, "\n");
}

static bool FileExists(const char *path) {
    struct stat st;
    return stat(path, &st) == 0;
}

static int MakeDir(const char *path) {
#ifdef _WIN32
    return _mkdir(path);
#else
    return mkdir(path, 0777);
#endif
}

// 途中のフォルダも作る(ドライブ名と先頭の区切りは作らない)
static bool MakeDirs(const char *dir) {
    char buf[PATH_MAX_LEN];
    snprintf(buf, sizeof(buf), "%s", dir);
    char *start = buf;
    if (((start[0] >= 'A' && start[0] <= 'Z') || (start[0] >= 'a' && start[0] <= 'z')) && start[1] == ':') start += 2;
    while (*start == '/' || *start == '\\') start++;
    if (*start == '\0') return true;
    for (char *p = start + 1; *p != '\0'; p++) {
        if (*p == '/' || *p == '\\') {
            char c = *p;
            *p = '\0';
            if (MakeDir(buf) != 0 && errno != EEXIST) return false;
            *p = c;
        }
    }
    return MakeDir(buf) == 0 || errno == EEXIST;
}

// path のフォルダの部分(最後の区切りまで。区切りがなければ ".")
static void DirName(const char *path, char *out, size_t n) {
    snprintf(out, n, "%s", path);
    char *slash = NULL;
    for (char *p = out; *p != '\0'; p++) {
        if (*p == '/' || *p == '\\') slash = p;
    }
    if (slash != NULL) *slash = '\0';
    else snprintf(out, n, ".");
}

// ファイルの1行目(改行と前後の空白を除く)。読めなければ false
static bool ReadFirstLine(const char *path, char *out, size_t n) {
    FILE *f = fopen(path, "r");
    if (f == NULL) return false;
    bool ok = fgets(out, (int)n, f) != NULL;
    fclose(f);
    if (!ok) return false;
    size_t len = strlen(out);
    while (len > 0 && (out[len - 1] == '\n' || out[len - 1] == '\r' || out[len - 1] == ' ')) out[--len] = '\0';
    return len > 0;
}

// コマンドを実行する。Windows の system() は cmd.exe を通るので、全体をもう一組の " で囲む
// (cmd.exe は、先頭が " のとき最初と最後の " を取り除くため)
static int RunShell(const char *cmd) {
    char buf[CMD_MAX_LEN];
#ifdef _WIN32
    snprintf(buf, sizeof(buf), "\"%s\"", cmd);
#else
    snprintf(buf, sizeof(buf), "%s", cmd);
#endif
    fflush(stdout);
    return system(buf);
}

// plant_sim の実行ファイルを探す
static bool FindPlantExe(const PlantOptions *o, char *out, size_t n) {
    if (o->exe != NULL && o->exe[0] != '\0') {
        snprintf(out, n, "%s", o->exe);
        return true;
    }
    const char *env = getenv("YUHO_PLANT_SIM");
    if (env != NULL && env[0] != '\0') {
        snprintf(out, n, "%s", env);
        return true;
    }
    char self_dir[PATH_MAX_LEN], local[PATH_MAX_LEN];
    DirName(o->self != NULL ? o->self : ".", self_dir, sizeof(self_dir));
    snprintf(local, sizeof(local), "%s/../plant_sim.local", self_dir); // build/ の1つ上 = tools/maze_sim
    return ReadFirstLine(local, out, n);
}

// out_dir/name のファイルから、prefix で始まる最後の行を取り出す
static bool FindLine(const char *path, const char *prefix, char *out, size_t n) {
    FILE *f = fopen(path, "r");
    if (f == NULL) return false;
    char line[1024];
    bool found = false;
    size_t plen = strlen(prefix);
    while (fgets(line, sizeof(line), f) != NULL) {
        if (strncmp(line, prefix, plen) == 0) {
            snprintf(out, n, "%s", line);
            found = true;
        }
    }
    fclose(f);
    if (found) {
        size_t len = strlen(out);
        while (len > 0 && (out[len - 1] == '\n' || out[len - 1] == '\r')) out[--len] = '\0';
    }
    return found;
}

// CSV の最後の行の最初の列(time_s)
static bool LastTime(const char *csv, double *t) {
    FILE *f = fopen(csv, "r");
    if (f == NULL) return false;
    char line[2048], last[2048] = "";
    while (fgets(line, sizeof(line), f) != NULL) {
        if (line[0] >= '0' && line[0] <= '9') snprintf(last, sizeof(last), "%s", line);
    }
    fclose(f);
    if (last[0] == '\0') return false;
    *t = atof(last);
    return true;
}

bool PlantLog_Run(const PlantOptions *o, const char *maze_path, const SearchTimeParams *sp, SearchAlgo algo,
                  float est_s, const char *out_dir, char *log_out, size_t log_len) {
    if (log_out != NULL && log_len > 0) log_out[0] = '\0';
    // --- 機体の操作の手順(RUN の SEARCH。app/search_run の SearchMenu_Run と同じ順) ---
    int iv = IndexOf(kSearchSpeeds, COUNT_OF(kSearchSpeeds), sp->v_mm_s);
    int ia = IndexOf(kAccels, COUNT_OF(kAccels), sp->accel_mm_s2);
    int it = IndexOf(kTurnSpeeds, COUNT_OF(kTurnSpeeds), sp->turn_v_mm_s);
    if (iv < 0 || ia < 0 || it < 0) {
        fprintf(stderr, "plant-log: the search speeds must be values the robot can choose (params.h SPEED_SELECT_*):\n");
        PrintChoices("SPEED (V)", kSearchSpeeds, COUNT_OF(kSearchSpeeds));
        PrintChoices("SLALOM (TURN_V)", kTurnSpeeds, COUNT_OF(kTurnSpeeds));
        PrintChoices("ACCEL", kAccels, COUNT_OF(kAccels));
        return false;
    }
    // sel N: 1番目の項目から N 段回して決める。MENU の1番目が RUN、RUN の1番目が SEARCH(app/mode_ui.c)
    char ops[256];
    snprintf(ops, sizeof(ops), "sel 0; sel 0; sel 0; sel %d; sel %d; sel %d; sel %d; sel %d; sel %d; hand; wait 3000",
             (int)sp->scope - 1, (algo == SEARCH_ALGO_ADACHI) ? 1 : 0, iv, ia, it, sp->slalom ? 0 : 1);

    // --- plant_sim ---
    char exe[PATH_MAX_LEN];
    if (!FindPlantExe(o, exe, sizeof(exe))) {
        fprintf(stderr, "plant-log: plant_sim not found. give --plant-exe PATH, set YUHO_PLANT_SIM, or write the path\n"
                        "  (e.g. M:/User/Desktop/school/club/sim/tools/plant_sim/build/plant_sim.exe) in tools/maze_sim/plant_sim.local\n");
        return false;
    }
    char cmd[CMD_MAX_LEN];
    if (o->build) {
        char exe_dir[PATH_MAX_LEN];
        DirName(exe, exe_dir, sizeof(exe_dir));
        printf("plant-log: building plant_sim (%s/../build.sh) ...\n", exe_dir);
        snprintf(cmd, sizeof(cmd), "sh \"%s/../build.sh\"", exe_dir);
        if (RunShell(cmd) != 0) {
            fprintf(stderr, "plant-log: plant_sim build failed\n");
            return false;
        }
    }
    char exe_win[PATH_MAX_LEN + 4];
    snprintf(exe_win, sizeof(exe_win), "%s.exe", exe); // Windows では .exe を省いて書いてもよい
    if (!FileExists(exe) && !FileExists(exe_win)) {
        fprintf(stderr, "plant-log: %s does not exist\n", exe);
        return false;
    }
    if (!MakeDirs(out_dir)) {
        fprintf(stderr, "plant-log: cannot make %s\n", out_dir);
        return false;
    }

    // 打ち切り: 見積もりの2倍 + 操作と待ちの分(最低 2 分)
    long limit_ms = (long)(est_s * 2.0f * 1000.0f) + 60000L;
    if (limit_ms < 120000L) limit_ms = 120000L;
    char sd_dir[PATH_MAX_LEN], stdout_path[PATH_MAX_LEN], stderr_path[PATH_MAX_LEN];
    snprintf(sd_dir, sizeof(sd_dir), "%s", out_dir);
    snprintf(stdout_path, sizeof(stdout_path), "%s/plant_stdout.txt", out_dir);
    snprintf(stderr_path, sizeof(stderr_path), "%s/plant_stderr.txt", out_dir);
    snprintf(cmd, sizeof(cmd),
             "\"%s\" run --maze \"%s\" --ops \"%s\" --out \"%s/truth.csv\" --out-every 10 --flash \"%s/flash.bin\" "
             "--sd-dir \"%s\" --limit-ms %ld %s > \"%s\" 2> \"%s\"",
             exe, maze_path, ops, out_dir, out_dir, sd_dir, limit_ms, (o->extra != NULL) ? o->extra : "",
             stdout_path, stderr_path);
    printf("plant-log: running plant_sim (ops: %s) ...\n", ops);
    RunShell(cmd);

    char end_line[512] = "", log_line[512] = "";
    FindLine(stderr_path, "end:", end_line, sizeof(end_line));
    printf("plant-log: %s\n", end_line[0] ? end_line : "(plant_sim did not report how it ended. see plant_stderr.txt)");
    if (!FindLine(stdout_path, "log: ", log_line, sizeof(log_line)) || strstr(log_line, "no SD") != NULL) {
        fprintf(stderr, "plant-log: no log was written (see %s)\n", stdout_path);
        return false;
    }

    // --- .bin → .csv(機体のログと同じ。tools/get_log.py) ---
    char bin[PATH_MAX_LEN], self_dir[PATH_MAX_LEN], get_log[PATH_MAX_LEN];
    snprintf(bin, sizeof(bin), "%s/%s", out_dir, log_line + 5); // "log: search/search_0001.bin"
    DirName(o->self != NULL ? o->self : ".", self_dir, sizeof(self_dir));
    snprintf(get_log, sizeof(get_log), "%s/../../get_log.py", self_dir); // tools/maze_sim/build → tools
    const char *python = getenv("PYTHON");
    snprintf(cmd, sizeof(cmd), "%s \"%s\" --bin2csv \"%s\"", (python != NULL && python[0]) ? python : "python",
             get_log, bin);
    bool csv_ok = FileExists(get_log) && RunShell(cmd) == 0;
    char csv[PATH_MAX_LEN];
    snprintf(csv, sizeof(csv), "%s", bin);
    size_t len = strlen(csv);
    if (len > 4) strcpy(csv + len - 4, ".csv");
    printf("plant-log: log %s\n", bin);
    double t_log;
    if (csv_ok && LastTime(csv, &t_log)) {
        // ログは手かざしの後から始まる(手を離してからの待ち約 1 秒を含む)。maze_sim の見積もりは走り出しからの時間
        printf("plant-log: csv %s (%.1f s of log; maze_sim estimate of the search %.1f s)\n", csv, t_log, (double)est_s);
    } else {
        csv_ok = false;
        printf("plant-log: could not make the csv. run: python tools/get_log.py --bin2csv \"%s\"\n", bin);
    }
    if (log_out != NULL && log_len > 0) snprintf(log_out, log_len, "%s", csv_ok ? csv : bin);
    return true;
}
