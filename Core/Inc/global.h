#ifndef INC_GLOBAL_H_
#define INC_GLOBAL_H_

#include <stdint.h>
#include <stdbool.h>

// ISR(interface/timer.c)がセンサー値を書き込み、main.cのデバッグ表示が読む。
// HAL型には依存しないため、logic層からもこのファイル経由で安全に参照できる。
extern volatile uint16_t ad_r, ad_fr, ad_fl, ad_l, vabt;

#endif
