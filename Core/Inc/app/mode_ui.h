#ifndef INC_MODEUI_H_
#define INC_MODEUI_H_


#include "global.h"

#include "interface/led.h"
#include "interface/uart.h"
#include "robot_state.h"

// 実行するモード(メニューの末端)。メニューの階層と並び順は mode_ui.c の表で決める。
//   RUN : SEARCH(Dijkstra) / SEARCH_ADACHI / FAST_RUN
//   TEST: SENSOR / SENSOR_LOG / VEL_PID / STRAIGHT / PIVOT / SLALOM / LED_TEST / PARTY
//   SD  : DUMP / DUMP_ALL / STREAM_TEST
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
// 操作と表示はモードの選択と同じ(右エンコーダで送り、ボタンで確定。n 番は LED n, n+1)。
// def に一番近い値から始める。UART には name・値・unit を出す。選んだ値を返す。
float ModeUI_SelectValue(const char *name, const char *unit, const float *values, uint8_t count, float def);

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