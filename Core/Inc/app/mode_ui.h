#ifndef INC_MODEUI_H_
#define INC_MODEUI_H_


#include "global.h"

#include "interface/led.h"
#include "interface/uart.h"

// 実行するモード。同定のブランチでは一段だけ(並び順がエンコーダで送る順。mode_ui.c の表で決める)。
//   1 E0(動き出し) / 2 E1(並進のステップ) / 3 E2(超信地旋回のステップ) / 4 E2S(超信地旋回の階段) /
//   5 E1S(前進のステップを電圧ごとに) / 6 E1V(並進の FF + 旋回の FB で走る検証) / 7 E1B(減速の同定) /
//   8 E1T(走りながら旋回のステップ) / 9 E1K0(V_d = 0 の曲がり具合) / 10 E1W(V_d の跳びで b_ω) /
//   11 SD_DUMP / 12 SD_DUMP_ALL
typedef enum {
    MODE_SYSID_E0 = 0,  // 電圧をゆっくり上げて、動き出す電圧を見る(app/sysid_mode)
    MODE_SYSID_E1,      // 並進のステップ(開ループ)
    MODE_SYSID_E2,      // 超信地旋回のステップ(開ループ)
    MODE_SYSID_E2S,     // 超信地旋回のステップを、電圧を上げながら1回の手かざしで続ける
    MODE_SYSID_E1S,     // 前進のステップを、手かざしごとに1つ電圧を上げて続ける
    MODE_SYSID_E1V,     // 同定したモデルの FF(並進)と旋回の FB で、台形の速度パターンを走る
    MODE_SYSID_E1B,     // FF で加速 → 一定 → 一定の電圧でブレーキ(減速の同定)
    MODE_SYSID_E1T,     // 走りながら左右の差をステップで入れる(旋回の同定)
    MODE_SYSID_E1K0,    // V_d = 0 で走ったときの曲がり具合 κ₀ を測る
    MODE_SYSID_E1W,     // V_d を ±A で切り替え、ω̇ の跳びから b_ω を測る
    MODE_SD_DUMP,
    MODE_SD_DUMP_ALL,
    MODE_COUNT
} RobotMode;

// 起動時のバッテリー残量をLEDバーで表示する(ブロッキング、hold_ms待つ)。
// FAILSAFE_LOW_VOLTAGE_V〜BATTERY_FULL_Vを、現在光るLED(1〜MODE_UI_BAR_LED_COUNT)
// の点灯本数に線形に割り当てる。しきい値未満は全消灯。
void ModeUI_ShowBattery(float vbat, uint32_t hold_ms);

// 起動時に一度呼ぶブロッキング処理。モードを右エンコーダの回転(MODE_SELECT_PULSES_PER_STEPごと)で送り、
// ボタン押下で確定する。番号は1始まりで、n 番はシフトレジスタの LED n, n+1 を点ける。
// 選択されたモードを返す。
RobotMode ModeUI_Select(void);

// モードを決めた後に、走りの設定の値(電圧など)を values の中から選ぶ(ブロッキング)。
// 操作はモードの選択と同じ(右エンコーダで送り、ボタンで確定)。表示は棒グラフで、n 番は LED1〜n を点ける。
// values は小さい順に並べること。1番から始める(def は count が 0 のときに返すだけ)。
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

// RAM に記録したログをSDカードへ保存し(バイナリ .bin)、結果をUARTに出す。
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
bool ModeUI_WaitHandStartOrClick(void);

// 選択されたモードを実行する。未知のモードは何もしない。
void ModeUI_Run(RobotMode mode);

#endif
