#include "app/stream_test.h"

#include "main.h"
#include "app/mode_ui.h"
#include "app/control_loop.h"
#include "app/failsafe.h"
#include "app/logger.h"
#include "app/run_log.h"
#include "interface/button.h"
#include "interface/sdcard.h"

#define STREAM_TEST_MAX_MS      120000u // 押さなくても、これで止める
#define STREAM_TEST_REPORT_MS   2000u   // 途中経過を出す間隔(printf の待ちも、走行中のメインの仕事の代わり)
#define STREAM_TEST_SHOW_MS     500u    // 書けたときに直結の LED を全部点ける時間

// 探索・最短走行と同じ列(全部の値 + イベント、34列)で流す
static void SetupLogger(uint32_t decimation) {
    RunLog_Setup("stream", "test", decimation);
}

// ボタンが押されたか(チャタリングを除いて)
static bool Pressed(void) {
    if (!Button_IsPressed()) return false;
    HAL_Delay(20);
    return Button_IsPressed();
}

void StreamTest_Run(void) {
    printf("STREAM TEST: stream log to SD while the main loop keeps polling (motors off)\r\n");
    printf("block %lu bytes, reserve %lu bytes\r\n",
           (unsigned long)LOGGER_STREAM_BLOCK_BYTES, (unsigned long)LOGGER_STREAM_RESERVE_BYTES);
    if (!SDCard_IsMounted()) {
        printf("SD not mounted\r\n");
    }

    static const uint32_t kDecimation[] = { 1u, 5u };
    uint8_t sel = 0;

    while (1) {
        printf("interval: %lu ms  (click: change, hand: START)\r\n", (unsigned long)kDecimation[sel]);
        LED_SetShiftPattern((uint16_t)(1u << sel));
        while (ModeUI_WaitHandStartOrClick()) {
            sel = (uint8_t)((sel + 1u) % 2u);
            printf("interval: %lu ms\r\n", (unsigned long)kDecimation[sel]);
            LED_SetShiftPattern((uint16_t)(1u << sel));
        }

        SetupLogger(kDecimation[sel]);
        char path[SDCARD_PATH_MAX];
        uint32_t t_open = HAL_GetTick();
        if (!Logger_StreamBegin(path, sizeof(path))) {
            printf("stream begin failed\r\n");
            continue;
        }
        printf("log: %s (open + reserve %lu ms)\r\n", path, (unsigned long)(HAL_GetTick() - t_open));

        Logger_Start();
        printf("streaming... press button to STOP\r\n");
        uint32_t t0 = HAL_GetTick();
        uint32_t last_report = t0;
        while (1) {
            if (FailSafe_IsTripped()) {
                Logger_StreamEnd(NULL);
                FailSafe_Halt();
            }
            Logger_StreamPoll();
            uint32_t now = HAL_GetTick();
            if (now - last_report >= STREAM_TEST_REPORT_MS) {
                last_report = now;
                LoggerStreamStats st;
                Logger_StreamGetStats(&st);
                printf("  %5.1f s: blocks %lu, dropped %lu, write avg %lu max %lu ms%s\r\n",
                       (float)(now - t0) / 1000.0f, (unsigned long)st.blocks, (unsigned long)st.dropped_rows,
                       (unsigned long)st.write_avg_ms, (unsigned long)st.write_max_ms,
                       Logger_StreamFailed() ? "  FAILED" : "");
            }
            if (Pressed() || now - t0 >= STREAM_TEST_MAX_MS) break;
        }
        uint32_t duration = HAL_GetTick() - t0;
        while (Button_IsPressed()) HAL_Delay(10);

        uint32_t dropped = 0;
        bool ok = Logger_StreamEnd(&dropped);
        LoggerStreamStats st;
        Logger_StreamGetStats(&st);
        uint32_t expected = duration / kDecimation[sel];
        printf("done %s: %.1f s, interval %lu ms -> about %lu rows expected\r\n",
               ok ? "OK" : "FAILED", (float)duration / 1000.0f, (unsigned long)kDecimation[sel],
               (unsigned long)expected);
        printf("  blocks %lu x %lu rows, dropped %lu rows, write avg %lu ms, max %lu ms\r\n",
               (unsigned long)st.blocks, (unsigned long)st.rows_per_block, (unsigned long)dropped,
               (unsigned long)st.write_avg_ms, (unsigned long)st.write_max_ms);
        if (ok) {
            LED_SetDirectPattern(LED_DIRECT_ALL);
            HAL_Delay(STREAM_TEST_SHOW_MS);
            LED_SetDirectPattern(0x00u);
        } else {
            LED_SetDirectPattern(MODE_UI_LED_SD_ERROR);
        }
    }
}
