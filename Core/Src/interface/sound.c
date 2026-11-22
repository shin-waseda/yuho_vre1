#include "interface/sound.h"

#include "main.h"
#include "tim.h"
#include "interface/fault_diag.h"

// ブザー(TIM3 CH2、PB5)。half_1(yozakura)の interface/sound を移したもの(2026-11-16)。
// TIM3 は分周なし(プリスケーラ 0)で数えるので、周期 = タイマーのクロック / hz。音量はデューティで決める
// (大きさは sin(π × デューティ) に比例し、50% が最大。yozakura で確かめた形)。
// 曲は SoundNote の並びで、SysTick(1ms)の Sound_Tick() が 1 音ずつ進める。

// 同じ高さの音が続いても切れて聞こえるように、各音の終わりを少し休む[ms]
#define SOUND_GAP_MS 15u

static const SoundNote *volatile s_cur = NULL; // 鳴らしている音(NULL = 止まっている)
static volatile uint32_t s_left_ms = 0u;        // その音の残り[ms]
static volatile uint32_t s_duty = SOUND_DUTY_PCT; // 曲の音量[%]
static uint32_t s_tim_clock_hz = 0u;            // TIM3 が数える速さ[Hz]

void Sound_Init(void) {
    // APB1 のタイマーは、APB1 を分周しているときは PCLK1 の 2 倍で数える(168MHz なら 84MHz)
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    s_tim_clock_hz = ((RCC->CFGR & RCC_CFGR_PPRE1) == RCC_CFGR_PPRE1_DIV1) ? pclk1 : 2u * pclk1;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0u);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

void Sound_Tone(uint32_t hz, uint32_t duty_pct) {
    if (hz == 0u || duty_pct == 0u || s_tim_clock_hz == 0u) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0u);
        return;
    }
    uint32_t period = s_tim_clock_hz / hz; // 16bit に収まる(84MHz なら 1282Hz 以上)
    if (period > 65536u) period = 65536u;
    __HAL_TIM_SET_AUTORELOAD(&htim3, period - 1u);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, period * duty_pct / 100u);
    __HAL_TIM_SET_COUNTER(&htim3, 0u);
}

// s_cur の音を鳴らし始める。曲の終わりなら止める
static void StartNote(void) {
    if (s_cur->ms == 0u) {
        s_cur = NULL;
        Sound_Tone(0u, 0u);
        return;
    }
    s_left_ms = s_cur->ms;
    Sound_Tone(s_cur->hz, s_duty);
}

void Sound_SetDuty(uint32_t duty_pct) {
    s_duty = (duty_pct > 50u) ? 50u : duty_pct;
}

void Sound_Tick(void) {
    if (s_cur == NULL) return;
    s_left_ms--;
    if (s_left_ms == SOUND_GAP_MS && s_cur->ms > 2u * SOUND_GAP_MS) {
        Sound_Tone(0u, 0u);
    }
    if (s_left_ms == 0u) {
        s_cur++;
        StartNote();
    }
}

void Sound_Play(const SoundNote *seq) {
    __disable_irq();
    s_cur = seq;
    StartNote();
    __enable_irq();
}

void Sound_Stop(void) {
    __disable_irq();
    s_cur = NULL;
    Sound_Tone(0u, 0u);
    __enable_irq();
}

bool Sound_IsBusy(void) {
    return s_cur != NULL;
}

void Sound_Wait(void) {
    while (Sound_IsBusy()) {
    }
}

// ---- 決まった音 ----

// モード選択: 番号をドレミ…(ハ長調)に割り当てる。
// 0 = ド(C7)、1 = レ、… 7 = 上のド(C8)、… 13 = さらに上のシ(B8)。14 から先は 0 に戻る。
static const uint16_t kModeScale[] = {
    NOTE_C7, NOTE_D7, NOTE_E7, NOTE_F7, NOTE_G7, NOTE_A7, NOTE_B7,
    NOTE_C8, NOTE_D8, NOTE_E8, NOTE_F8, NOTE_G8, NOTE_A8, NOTE_B8,
};
#define MODE_SCALE_N ((uint32_t)(sizeof(kModeScale) / sizeof(kModeScale[0])))
#define MODE_BEEP_MS 120u

void Sound_Mode(uint8_t index) {
    static SoundNote beep[2]; // 鳴らしている間も読まれるので static
    Sound_Stop();
    beep[0].hz = kModeScale[index % MODE_SCALE_N];
    beep[0].ms = MODE_BEEP_MS;
    beep[1].hz = 0u;
    beep[1].ms = 0u;
    Sound_Play(beep);
}

// 起動音(yozakura と同じ: 電圧の 2 音 + 前回の電源が落ちた原因の 2 音)。
//
// 電圧の 2 音: 0.1V 単位に丸めて、1 の位と小数第 1 位をモードと同じ音階で鳴らす。
//   0 = ド(C7)、1 = レ、2 = ミ、3 = ファ、4 = ソ、5 = ラ、6 = シ、7 = ド(C8)、8 = レ、9 = ミ
//   例: 8.2V → レ(D8)・ミ、7.6V → ド(C8)・シ
//
// 原因の 2 音(上がる・同じ = ふつう、下がる = 異常):
//   ウォッチドッグ(IWDG、WWDG)    ド(C8)→ ド(C7)  オクターブ下がる
//   低電力モードのリセット(LPWR)  ミ(E8)→ ミ(E7)  オクターブ下がる
//   電源を入れた(POR)             ド → ソ          上がる
//   電圧の低下(BOR だけ)          ソ → ミ          下がる
//   ソフトのリセット(書き込み後・HardFault の後) ソ → ド(C8)  上がる
//   RESET ボタン・NRST のノイズ(PIN だけ)       ソ → ソ      同じ
// POR やソフトのリセットでも PIN と BOR の印が一緒に立つので、上の順に先に当たった方とする。
#define STARTUP_NOTE_MS 140u
#define STARTUP_GAP_MS  200u

void Sound_Startup(float vbat_v, uint32_t reset_flags) {
    static SoundNote seq[6]; // 鳴らしている間も読まれるので static
    uint32_t d = (vbat_v > 0.0f) ? (uint32_t)(vbat_v * 10.0f + 0.5f) : 0u; // 0.1V 単位
    uint32_t ones = d / 10u;
    if (ones > 9u) ones = 9u;
    uint16_t r1, r2;
    if (reset_flags & (FAULT_RESET_IWDG | FAULT_RESET_WWDG)) {
        r1 = NOTE_C8; r2 = NOTE_C7;
    } else if (reset_flags & FAULT_RESET_LPWR) {
        r1 = NOTE_E8; r2 = NOTE_E7;
    } else if (reset_flags & FAULT_RESET_POR) {
        r1 = NOTE_C7; r2 = NOTE_G7;
    } else if (reset_flags & FAULT_RESET_BOR) {
        r1 = NOTE_G7; r2 = NOTE_E7;
    } else if (reset_flags & FAULT_RESET_SFT) {
        r1 = NOTE_G7; r2 = NOTE_C8;
    } else {
        r1 = NOTE_G7; r2 = NOTE_G7;
    }
    Sound_Stop();
    seq[0] = (SoundNote){ kModeScale[ones], STARTUP_NOTE_MS };
    seq[1] = (SoundNote){ kModeScale[d % 10u], STARTUP_NOTE_MS };
    seq[2] = (SoundNote){ NOTE_REST, STARTUP_GAP_MS };
    seq[3] = (SoundNote){ r1, STARTUP_NOTE_MS };
    seq[4] = (SoundNote){ r2, STARTUP_NOTE_MS };
    seq[5] = (SoundNote){ 0u, 0u };
    Sound_Play(seq);
}

// ゴール、走りの終わり(yozakura と同じ「レミレミレッラ レミファ#ッソッラー」)。
// 「ッ」は 1 拍休む、「ー」はのばす、空白は 2 拍休む。1 拍 = FINISH_BEAT_MS
#define FINISH_BEAT_MS 110u
#define BEAT(n) ((uint16_t)((n) * FINISH_BEAT_MS))
static const SoundNote kFinish[] = {
    { NOTE_D7, BEAT(1) }, { NOTE_E7, BEAT(1) }, { NOTE_D7, BEAT(1) }, { NOTE_E7, BEAT(1) },
    { NOTE_D7, BEAT(1) }, { NOTE_REST, BEAT(1) }, { NOTE_A7, BEAT(1) },
    { NOTE_REST, BEAT(2) },
    { NOTE_D7, BEAT(1) }, { NOTE_E7, BEAT(1) }, { NOTE_FS7, BEAT(1) }, { NOTE_REST, BEAT(1) },
    { NOTE_G7, BEAT(1) }, { NOTE_REST, BEAT(1) }, { NOTE_A7, BEAT(4) },
    { 0u, 0u },
};

void Sound_Finish(void) {
    Sound_Play(kFinish);
}

// 電池電圧の低下: 高い音(E8)でピピピ
static const SoundNote kBattLow[] = {
    { NOTE_E8, 80u }, { NOTE_REST, 50u },
    { NOTE_E8, 80u }, { NOTE_REST, 50u },
    { NOTE_E8, 80u },
    { 0u, 0u },
};

void Sound_BattLow(void) {
    Sound_Play(kBattLow);
}

// 選ぶ所の区切り: ラ(A7)でピピピ(電池の警告の E8 より低くして聞き分ける)
static const SoundNote kReady[] = {
    { NOTE_A7, 70u }, { NOTE_REST, 60u },
    { NOTE_A7, 70u }, { NOTE_REST, 60u },
    { NOTE_A7, 70u },
    { 0u, 0u },
};

void Sound_Ready(void) {
    Sound_Play(kReady);
}

// ---- ログ ----

static const SoundNote kLogStart[] = {
    { NOTE_G7, 60u }, { NOTE_REST, 40u }, { NOTE_G7, 60u },
    { 0u, 0u },
};

static const SoundNote kLogSaved[] = {
    { NOTE_C7, 90u }, { NOTE_G7, 90u }, { NOTE_C8, 180u },
    { 0u, 0u },
};

static const SoundNote kLogSent[] = {
    { NOTE_C8, 80u }, { NOTE_E8, 80u }, { NOTE_G8, 160u },
    { 0u, 0u },
};

static const SoundNote kError[] = {
    { NOTE_G7, 250u }, { NOTE_C7, 400u },
    { 0u, 0u },
};

void Sound_LogStart(void) {
    Sound_Play(kLogStart);
}

void Sound_LogSaved(void) {
    Sound_Play(kLogSaved);
}

void Sound_LogSent(void) {
    Sound_Play(kLogSent);
}

void Sound_Error(void) {
    Sound_Play(kError);
}
