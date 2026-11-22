#ifndef INC_SOUND_H_
#define INC_SOUND_H_


#include "global.h"
#include "params.h"

// ブザー(TIM3 CH2、PB5)で音を鳴らす。half_1(yozakura)の interface/sound を移したもの(2026-11-16)。
// 鳴らしている間も止まらない(SysTick の 1ms ごとに Sound_Tick() で次の音へ進む)。
// TIM3 は FAN(CH1)と周期を共有しているので、音を鳴らすと FAN の PWM の周期も変わる(今は FAN を使っていない)。

// 音の高さ[Hz](7 オクターブ目。yozakura のブザーは共振 4kHz の近くで大きく鳴った)
#define NOTE_C7  2093
#define NOTE_D7  2349
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_A7  3520
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_D8  4699
#define NOTE_E8  5274
#define NOTE_F8  5588
#define NOTE_G8  6272
#define NOTE_A8  7040
#define NOTE_B8  7902
#define NOTE_REST 0

// 1 音。hz = 0 は休み、ms = 0 は曲の終わり
typedef struct {
    uint16_t hz;
    uint16_t ms;
} SoundNote;

// TIM3 の CH2 の PWM を始める(MX_TIM3_Init の後に1回)
void Sound_Init(void);
// SysTick(1ms)から呼ぶ
void Sound_Tick(void);

void Sound_Tone(uint32_t hz, uint32_t duty_pct); // 直接鳴らす(hz = 0 で止める)
void Sound_SetDuty(uint32_t duty_pct);           // 曲の音量[%](初期値は SOUND_DUTY_PCT。50 が一番大きい)
void Sound_Play(const SoundNote *seq);           // 曲を鳴らし始める(前の曲は止める)。seq は鳴り終わるまで残しておくこと
void Sound_Stop(void);
bool Sound_IsBusy(void);
void Sound_Wait(void); // 鳴り終わるまで待つ

// ---- 決まった音 ----
void Sound_Mode(uint8_t index);                         // モード・値の選択: 番号(0 始まり)を音階に割り当てる
void Sound_Startup(float vbat_v, uint32_t reset_flags); // 起動音: 電池電圧の 2 音 + リセットの原因の 2 音(FAULT_RESET_*)
void Sound_Finish(void);   // ゴール、走りの終わり
void Sound_BattLow(void);  // 電池電圧の低下(高い警告音)
void Sound_Ready(void);    // 選ぶ所の区切り(最短走行で速さを選び終えた): ラでピピピ(短く 3 回)
void Sound_LogStart(void); // 記録を始めた: ソ・ソ(短く 2 回)
void Sound_LogSaved(void); // SD に保存し終えた: ド・ソ・上のド(上がる)
void Sound_LogSent(void);  // PC に送り終えた: 上のド・ミ・ソ(高く上がる)
void Sound_Error(void);    // 失敗した: ソ → ド(低く下がる、長め)

#endif
