#include "app/sensor_log.h"

#include "main.h"
#include "app/mode_ui.h"
#include "app/control_loop.h"
#include "app/failsafe.h"
#include "app/logger.h"
#include "logic/wall_sense.h"

// 記録する長さ[ms]。止まった状態の値のばらつきを見るのに足りる長さ。
#define SENSOR_LOG_MS 3000

// ボタンを離してから記録を始めるまでの待ち[ms](ボタンを押した手の影や振動が入らないように)。
#define SENSOR_LOG_START_DELAY_MS 500

typedef struct {
    uint32_t sum;
    uint16_t min;
    uint16_t max;
} SensorStat;

static void StatReset(SensorStat *s) {
    s->sum = 0;
    s->min = 0xFFFFu;
    s->max = 0;
}

static void StatAdd(SensorStat *s, uint16_t v) {
    s->sum += v;
    if (v < s->min) s->min = v;
    if (v > s->max) s->max = v;
}

static void StatPrint(const char *name, const SensorStat *s, uint32_t n) {
    printf("  %-2s: mean %6.1f  min %4u  max %4u\r\n",
           name, (n > 0) ? (float)s->sum / (float)n : 0.0f, s->min, s->max);
}

static void SetupLogger(void) {
    const ControlDebug *d = App_GetControlDebug();

    Logger_Init();
    Logger_SetDirName("sensor");
    Logger_SetFileName("wall");
    Logger_AddField("ad_l", &d->ad_l);
    Logger_AddField("ad_fl", &d->ad_fl);
    Logger_AddField("ad_fr", &d->ad_fr);
    Logger_AddField("ad_r", &d->ad_r);
    Logger_AddField("vbat", &d->vbat);
    Logger_SetDuration(SENSOR_LOG_MS);
}

// SENSOR_LOG_MS のあいだ記録しながら、メイン側でも1msごとに読んで統計を取る
// (Logger のバッファは中身を読む関数がないので、表示用の統計は別に集める)。
static void RecordOnce(void) {
    SensorStat st_l, st_fl, st_fr, st_r;
    StatReset(&st_l);
    StatReset(&st_fl);
    StatReset(&st_fr);
    StatReset(&st_r);
    uint32_t n = 0;

    Logger_Start();
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < SENSOR_LOG_MS) {
        if (FailSafe_IsTripped()) {
            Logger_Stop();
            FailSafe_Halt();
        }
        StatAdd(&st_l, ad_l);
        StatAdd(&st_fl, ad_fl);
        StatAdd(&st_fr, ad_fr);
        StatAdd(&st_r, ad_r);
        n++;
        HAL_Delay(1);
    }
    Logger_Stop();

    printf("sensor (%lu ms, %lu reads):\r\n", (unsigned long)SENSOR_LOG_MS, (unsigned long)n);
    StatPrint("L", &st_l, n);
    StatPrint("FL", &st_fl, n);
    StatPrint("FR", &st_fr, n);
    StatPrint("R", &st_r, n);

    // 平均の値で壁の有無を判定して見せる(しきい値の確かめ用)
    if (n > 0) {
        WallSensorValues v = {
            .l = (uint16_t)(st_l.sum / n),
            .fl = (uint16_t)(st_fl.sum / n),
            .fr = (uint16_t)(st_fr.sum / n),
            .r = (uint16_t)(st_r.sum / n),
        };
        WallObservation obs = WallSense_Judge(v);
        printf("walls: left %s  front %s  right %s  (FL+FR = %u)\r\n",
               obs.left ? "YES" : "no", obs.front ? "YES" : "no", obs.right ? "YES" : "no",
               (unsigned)(v.fl + v.fr));
    }
}

void SensorLog_Run(void) {
    printf("SENSOR LOG: record wall sensors for %lu ms per press (motors off)\r\n",
           (unsigned long)SENSOR_LOG_MS);

    SetupLogger();

    while (1) {
        printf("press button to LOG\r\n");
        ModeUI_WaitClick();
        HAL_Delay(SENSOR_LOG_START_DELAY_MS); // ボタンを押した手を離す時間

        RecordOnce();
        printf("done: %lu samples x %lu fields\r\n",
               (unsigned long)Logger_SampleCount(), (unsigned long)Logger_FieldCount());
        SdSaveResult saved = ModeUI_SaveLogToSD(); // SDがあれば自動で保存

        printf("press button to DUMP\r\n");
        if (saved == SD_SAVE_FAILED) {
            ModeUI_WaitClickBlinking(MODE_UI_LED_SD_ERROR); // 保存の失敗を左後ろのLEDの点滅で知らせながら待つ
        } else {
            ModeUI_WaitClick();
        }
        Logger_Dump();
    }
}
