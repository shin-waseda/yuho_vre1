#include "main.h"
#include <stdio.h>
#include "interface/fault_diag.h"
#include "interface/motor.h"
#include "interface/led.h"

#define FAULT_MAGIC 0x46415554u // "FAUT"

// HardFault のときに残す記録。CCMRAM に置くので、リセットしても残る(電源を切ると消える)。
typedef struct {
    uint32_t magic;
    uint32_t pc, lr, psr, cfsr, hfsr, mmfar, bfar, sp;
    uint32_t check; // magic 以外を足した値の反転(電源投入直後のでたらめな値と見分ける)
} FaultRecord;

static FaultRecord s_record __attribute__((section(".ccmram")));
static FaultDiagInfo s_boot;

// スタックとして読んでよい範囲(SRAM 128KB と CCMRAM 64KB)
#define SRAM_START 0x20000000u
#define SRAM_END   0x20020000u
#define CCM_START  0x10000000u
#define CCM_END    0x10010000u

#define LED_LEFT_SIDE  (LED_LEFT | LED_FRONT_LEFT | LED_REAR_LEFT)
#define LED_RIGHT_SIDE (LED_RIGHT | LED_FRONT_RIGHT | LED_REAR_RIGHT)
#define FAULT_BLINK_MS 100u
#define FAULT_SHOW_MS  3000u // HardFault のとき、リセットする前に点滅させる時間

static uint32_t RecordCheck(const FaultRecord *r) {
    return ~(r->pc + r->lr + r->psr + r->cfsr + r->hfsr + r->mmfar + r->bfar + r->sp);
}

static bool InStack(uint32_t addr, uint32_t bytes) {
    return (addr >= SRAM_START && addr + bytes <= SRAM_END) ||
           (addr >= CCM_START && addr + bytes <= CCM_END);
}

// HardFault の中では SysTick が進まない(HAL_Delay が使えない)ので、CPU のサイクル数で待つ
static void WaitMsCycles(uint32_t ms) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = (SystemCoreClock / 1000u) * ms;
    while (DWT->CYCCNT - start < cycles) {
    }
}

void FaultDiag_HardFault(uint32_t *sp, uint32_t exc_return) {
    // まずモーターを止める(PWM はタイマーが出し続けるので、止めないと回り続ける)
    Motor_Stop();
    Motor_Disable();

    // 例外に入ったときに CPU が積んだ r0〜r3, r12, lr, pc, psr の場所を探す
    uint32_t *frame = NULL;
    if ((exc_return & 0xFFFFFFE0u) == 0xFFFFFFE0u) {
        if (exc_return & 4u) {
            frame = (uint32_t *)__get_PSP();
        } else {
            // HardFault_Handler の始めで lr(= exc_return)を積んでいれば、そのすぐ上が CPU の積んだ所
            frame = sp;
            for (uint32_t i = 0; i < 16u && InStack((uint32_t)&sp[i], 4u); i++) {
                if (sp[i] == exc_return) {
                    frame = &sp[i + 1u];
                    break;
                }
            }
        }
    }

    FaultRecord r = { 0 };
    if (frame != NULL && InStack((uint32_t)frame, 32u)) {
        r.lr = frame[5];
        r.pc = frame[6];
        r.psr = frame[7];
        r.sp = (uint32_t)frame;
    }
    r.cfsr = SCB->CFSR;
    r.hfsr = SCB->HFSR;
    r.mmfar = SCB->MMFAR;
    r.bfar = SCB->BFAR;
    r.check = RecordCheck(&r);
    r.magic = FAULT_MAGIC;
    s_record = r;

    // 左右交互の速い点滅(フェイルセーフの点滅と見分けるため)
    for (uint32_t t = 0; t < FAULT_SHOW_MS; t += 2u * FAULT_BLINK_MS) {
        LED_SetDirectPattern(LED_LEFT_SIDE);
        WaitMsCycles(FAULT_BLINK_MS);
        LED_SetDirectPattern(LED_RIGHT_SIDE);
        WaitMsCycles(FAULT_BLINK_MS);
    }
    NVIC_SystemReset();
}

void FaultDiag_ReadAtBoot(void) {
    s_boot.reset_flags = (RCC->CSR >> 24) & 0xFEu;
    RCC->CSR |= RCC_CSR_RMVF; // 次のリセットの原因だけが残るよう消す

    const FaultRecord *r = &s_record;
    s_boot.had_fault = (r->magic == FAULT_MAGIC) && (r->check == RecordCheck(r));
    if (s_boot.had_fault) {
        s_boot.pc = r->pc;
        s_boot.lr = r->lr;
        s_boot.psr = r->psr;
        s_boot.cfsr = r->cfsr;
        s_boot.hfsr = r->hfsr;
        s_boot.mmfar = r->mmfar;
        s_boot.bfar = r->bfar;
        s_boot.sp = r->sp;
    }
    s_record.magic = 0; // 次の起動で同じ記録を読まないように
}

const FaultDiagInfo *FaultDiag_GetBootInfo(void) {
    return &s_boot;
}

void FaultDiag_Print(void) {
    static const char *const kNames[8] = { "", "BOR", "PIN", "POR", "SOFT", "IWDG", "WWDG", "LPWR" };
    printf("reset cause:");
    for (int i = 1; i < 8; i++) {
        if (s_boot.reset_flags & (1u << i)) printf(" %s", kNames[i]);
    }
    printf("\r\n");
    if (!s_boot.had_fault) return;

    printf("!! HARDFAULT before this reset: pc=0x%08lX lr=0x%08lX psr=0x%08lX sp=0x%08lX\r\n",
           (unsigned long)s_boot.pc, (unsigned long)s_boot.lr, (unsigned long)s_boot.psr,
           (unsigned long)s_boot.sp);
    printf("   cfsr=0x%08lX hfsr=0x%08lX mmfar=0x%08lX bfar=0x%08lX\r\n",
           (unsigned long)s_boot.cfsr, (unsigned long)s_boot.hfsr,
           (unsigned long)s_boot.mmfar, (unsigned long)s_boot.bfar);
    for (int i = 0; i < 10; i++) { // 約2秒、HardFault のときと同じ点滅
        LED_SetDirectPattern(LED_LEFT_SIDE);
        HAL_Delay(FAULT_BLINK_MS);
        LED_SetDirectPattern(LED_RIGHT_SIDE);
        HAL_Delay(FAULT_BLINK_MS);
    }
    LED_SetDirectPattern(0x00u);
}
