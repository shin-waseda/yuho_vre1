#ifndef INC_MODEUI_H_
#define INC_MODEUI_H_


#include "global.h"

#include "interface/led.h"
#include "interface/uart.h"
#include "robot_state.h"

// 実行するモード(メニューの末端)。メニューの階層と並び順は mode_ui.c の表で決める。
//   RUN : SEARCH(地図・行き先・アルゴリズム・速さなどを選ぶ) / FAST(最短走行) /
//         TEST(ログ取りの走行: FAST_SWEEP / SEARCH_SPIN / FAST_BANDS / LONG_LOG) / AUTO(自立賞)
//   TEST: SENSOR / SENSOR_LOG / VEL_PID / STRAIGHT / PIVOT / SLALOM / LED_TEST / PARTY
//   SD  : DUMP / DUMP_ALL / STREAM_TEST
//   4〜9: ログ取りのモードへのショートカット(選ぶとすぐ決まる。FAST_SWEEP 以外は値も params.h の SHORTCUT_* で決まっていて選ばない)
//         4 STRAIGHT_SWEEP(v)，5 STRAIGHT_SWEEP(acc)，6 SLALOM_SWEEP(low)，7 SLALOM_SWEEP(high)，8 FAST_SWEEP，9 LONG_LOG，
//         10 SENSOR_SPIN
typedef enum {
    MODE_SENSOR = 0,    // センサー・ジャイロ・エンコーダの値を表示し続ける
    MODE_SENSOR_LOG,    // 壁センサーの値を数秒ぶん記録する(止まった状態)
    MODE_VEL_PID,
    MODE_STRAIGHT_TEST,
    MODE_PIVOT_TEST,
    MODE_SLALOM_TEST,   // スラローム(小回り 90°)の試験
    MODE_LED_TEST,
    MODE_PARTY,         // 宴会芸(床を回されても同じ方向を向き続ける)
    MODE_SD_DUMP,
    MODE_SD_DUMP_ALL,
    MODE_STREAM_TEST,   // 走りながら SD へ流すログの試験(モーターは動かさない)
    MODE_SEARCH,        // 探索走行(Dijkstra)
    MODE_SEARCH_ADACHI, // 探索走行(足立法)
    MODE_FAST_RUN,      // 最短走行(探索で flash に残した地図を使う)
    MODE_LONG_LOG,      // 長い走行(探索と最短走行を速さを変えて続けて走り、ログを取る)
    MODE_SLALOM_SWEEP,  // 小回りの連続の試験(速さを上げながら右で行って左で戻る)
    MODE_STRAIGHT_SWEEP, // 直進の連続の試験(速さと加速度を上げながら行って戻る)
    MODE_FAST_SWEEP,    // 最短走行の連続(速さ・加速度・小回りの速さを変えて続けて走り、毎回スタートへ戻る)
    MODE_SENSOR_SPIN,   // 区画の真ん中で回りながら壁センサーを記録する(plant_sim の壁センサーのモデル用)
    MODE_SEARCH_SPIN,   // 探索(Dijkstra)で、初めて入った区画ごとに真ん中で回る(壁センサーのモデル用)
    MODE_FAST_BANDS,    // 速度帯の最短走行(速度帯を順に、行きと帰りを自動でくり返す)
    MODE_RUN_TEST,      // RUN の中の TEST(ログ取りの走行: FAST_SWEEP / SEARCH_SPIN / FAST_BANDS / LONG_LOG から選ぶ)
    MODE_AUTONOMOUS,    // 自立賞(探索 → 最短走行と帰り道を何本か。触らずに最後まで)
    MODE_COUNT
} RobotMode;

// 起動時のバッテリー残量をLEDバーで表示する(ブロッキング、hold_ms待つ)。
// FAILSAFE_LOW_VOLTAGE_V〜BATTERY_FULL_Vを、現在光るLED(1〜MODE_UI_BAR_LED_COUNT)
// の点灯本数に線形に割り当てる。しきい値未満は全消灯。
void ModeUI_ShowBattery(float vbat, uint32_t hold_ms);

// 起動時に一度呼ぶブロッキング処理。まず一番上の階層(RUN / TEST / SD)を選び、
// 次にその中のモードを選ぶ(上の階層には戻らない。戻るにはリセット)。
// 中のモードが1つだけの階層は、選んだ時点でそのモードに決まる。
// どちらも右エンコーダの回転(MODE_SELECT_PULSES_PER_STEPごと)で送り、
// ボタン押下で確定する。番号は1始まりで、n 番はシフトレジスタの LED n, n+1 を点ける。
// 選択されたモードを返す。
RobotMode ModeUI_Select(void);

// モードを決めた後に、走りの設定の値(速さなど)を values の中から選ぶ(ブロッキング)。
// 操作はモードの選択と同じ(右エンコーダで送り、ボタンで確定)。表示は棒グラフで、n 番は LED1〜n を点ける。
// values は遅い順(小さい順)に並べること。1番から始める(def は count が 0 のときに返すだけ)。
// UART には name・値・unit を出す。選んだ値を返す。
float ModeUI_SelectValue(const char *name, const char *unit, const float *values, uint8_t count, float def);
// ModeUI_SelectValue と同じだが、start 番目(0 始まり)から始める(回さずに決めるとその値になる)
float ModeUI_SelectValueFrom(const char *name, const char *unit, const float *values, uint8_t count, uint8_t start,
                             float def);

// ModeUI_Select で選ばれたモード(ログに残す用)。
RobotMode ModeUI_CurrentMode(void);

typedef enum {
    SD_SAVE_SKIPPED, // SDカードがない(マウントされていない)ので保存しなかった
    SD_SAVE_OK,
    SD_SAVE_FAILED,
} SdSaveResult;

// 試験モードの走行後、記録したログをSDカードへ保存し(バイナリ .bin)、結果をUARTに出す。
// 成功ならマイコン直結のLED全部を0.5秒点灯して消す(ブロッキング)。
// 失敗のときはLEDを変えずに SD_SAVE_FAILED を返すので、呼び出し側が
// 次のボタン待ちを ModeUI_WaitClickBlinking(MODE_UI_LED_SD_ERROR) にして知らせる。
SdSaveResult ModeUI_SaveLogToSD(void);

// SDへの保存の失敗を知らせる直結LED(左後ろ = LED_6)
#define MODE_UI_LED_SD_ERROR LED_REAR_LEFT

// ボタンが押されて離されるまで待つ(チャタリング除去付き)。
// 待っている間にフェイルセーフが発動したらFailSafe_Halt()へ入る(戻らない)。
void ModeUI_WaitClick(void);

// ModeUI_WaitClick() と同じだが、待っている間マイコン直結のLED(leds のビット。led.h の
// LED_FRONT_RIGHT など)を0.1秒周期で点滅させる(エラー表示)。押されたらLEDを消して戻る。
void ModeUI_WaitClickBlinking(uint8_t leds);

// 非接触スタート。左前の壁センサー(FL)に手をかざして(SENSOR_START_THRESHOLD を
// SENSOR_START_HOLD_MS 続けて超える)、離すまで待つ。かざしている間は左前の直結LEDを点ける。
// 待っている間にフェイルセーフが発動したらFailSafe_Halt()へ入る(戻らない)。
void ModeUI_WaitHandStart(void);

// ボタンのクリックか、非接触スタート(ModeUI_WaitHandStart と同じ手かざし)のどちらかを待つ。
// クリックなら true(離すまで待ってから)、手をかざして離したなら false を返す。
// 走る前に設定を選ぶ(クリックで切り替え、手かざしで走り出す)のに使う。
bool ModeUI_WaitHandStartOrClick(void);

// 選択されたモードを実行する。未知のモードは何もしない。
void ModeUI_Run(RobotMode mode);

#endif