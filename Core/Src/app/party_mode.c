#include "app/party_mode.h"

#include "main.h"
#include "app/mode_ui.h"
#include "app/control_loop.h"
#include "app/failsafe.h"
#include "app/logger.h"
#include "app/run_log.h"
#include "interface/button.h"
#include "interface/sdcard.h"

// 手を離してから向きを保ち始めるまでの待ち[ms](手を離す時間)。
#define PARTY_START_DELAY_MS 1000

// 記録の間隔[tick]。走りながら SD へ流すので、ボタンで止めるまで全部残る(探索と同じ 5ms ごと)。
#define PARTY_LOG_DECIMATION 5
// 途中経過(流したブロックの数・捨てた行の数)を UART に出す間隔
#define PARTY_REPORT_MS      10000u
#define PARTY_SAVED_LIGHT_MS 500u // 書けたときに直結の LED を全部点ける時間

// 全部の値 + イベント(app/run_log)
static void SetupLogger(void) {
    RunLog_Setup("party", "hold", PARTY_LOG_DECIMATION);
}

static bool Pressed(void) {
    if (!Button_IsPressed()) return false;
    HAL_Delay(20);
    return Button_IsPressed();
}

static void PrintStats(const char *head, uint32_t elapsed_ms) {
    LoggerStreamStats st;
    Logger_StreamGetStats(&st);
    printf("%s %.0f s: blocks %lu, dropped %lu rows, write avg %lu max %lu ms%s\r\n",
           head, (float)elapsed_ms / 1000.0f, (unsigned long)st.blocks, (unsigned long)st.dropped_rows,
           (unsigned long)st.write_avg_ms, (unsigned long)st.write_max_ms,
           Logger_StreamFailed() ? "  SD FAILED" : "");
}

void PartyMode_Run(void) {
    printf("PARTY: keep heading (gyro) and position (encoders), log streamed to SD until STOP\r\n");
    printf("ANGULAR: %s ANGULAR_KP=%.2f ANGLE_KP=%.2f, limit %.0f dps\r\n",
           ANGULAR_CONTROL_ENABLE ? "ON" : "OFF", ANGULAR_KP, ANGLE_KP,
           ANGULAR_CORR_LIMIT_RAD_S * 180.0f / 3.14159265f);

    if (FailSafe_IsTripped()) FailSafe_Halt(); // 起動時の低電圧など

    App_SetPositionHold(true); // 向きに加えて、前後の位置も保つ(車輪で測った距離を、有効にした所に戻す)

    while (1) {
        printf("hold hand over front-left sensor to START\r\n");
        ModeUI_WaitHandStart();
        HAL_Delay(PARTY_START_DELAY_MS - GYRO_RECAL_MS); // 手を離す時間
        // 機体が止まっている間に、ジャイロのゼロ点を測り直す(向きがゆっくり回っていかないように)
        float gyro_offset = App_RecalibrateGyroZ(GYRO_RECAL_MS);
        printf("gyro z offset: %.1f\r\n", gyro_offset);

        SetupLogger();
        char path[SDCARD_PATH_MAX];
        bool logging = Logger_StreamBegin(path, sizeof(path));
        printf("log: %s\r\n", logging ? path : "(no SD, not saved)");

        // 有効にした時点の向きを保つ(目標の角速度は0のまま)
        Logger_Start(); // ここから記録(手かざしで始めた走りの始まり)
        Logger_Event(LOG_EV_MODE, (float)ModeUI_CurrentMode(), 0.0f, 0.0f, 0.0f, 0.0f);
        Logger_Event(LOG_EV_HAND_START, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        Logger_Event(LOG_EV_GYRO_RECAL, gyro_offset, 0.0f, 0.0f, 0.0f, 0.0f);
        App_SetTargetVelocity(0.0f);
        App_ControlLoop_SetEnabled(true);
        printf("holding heading. press button to STOP\r\n");

        uint32_t t0 = HAL_GetTick();
        uint32_t last_report = t0;
        while (1) {
            if (FailSafe_IsTripped()) {
                Logger_Event(LOG_EV_FAILSAFE, (float)FailSafe_GetCause(), FailSafe_GetFilteredVoltage(),
                             0.0f, 0.0f, 0.0f);
                HAL_Delay(PARTY_LOG_DECIMATION * 2u); // イベントが記録の行に入るのを待つ
                if (logging) Logger_StreamEnd(NULL); // ここまでのログは残す
                FailSafe_Halt();
            }
            if (logging) Logger_StreamPoll();
            uint32_t now = HAL_GetTick();
            if (logging && now - last_report >= PARTY_REPORT_MS) {
                last_report = now;
                PrintStats(" ", now - t0);
            }
            if (Pressed()) break;
        }
        Logger_Event(LOG_EV_CLICK, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        App_ControlLoop_SetEnabled(false);
        HAL_Delay(PARTY_LOG_DECIMATION * 2u); // イベントが記録の行に入るのを待つ
        while (Button_IsPressed()) HAL_Delay(10);

        if (logging) {
            bool ok = Logger_StreamEnd(NULL);
            PrintStats(ok ? "stopped, log saved:" : "stopped, log save FAILED:", HAL_GetTick() - t0);
            LED_SetDirectPattern(ok ? LED_DIRECT_ALL : MODE_UI_LED_SD_ERROR);
            HAL_Delay(PARTY_SAVED_LIGHT_MS);
            LED_SetDirectPattern(ok ? 0x00u : MODE_UI_LED_SD_ERROR);
        } else {
            Logger_Stop();
            printf("stopped\r\n");
        }
    }
}
