#ifndef INC_FAULT_DIAG_H_
#define INC_FAULT_DIAG_H_

#include "global.h"

// 止まった原因を後から調べるための仕込み。
// - HardFault のとき: モーターを止め、止まった場所(PC など)と原因のレジスタ(CFSR など)を CCMRAM に残し、
//   LED を左右交互に速く点滅させてから、自分でリセットする(電源を切ると CCMRAM は消えるため)。
// - 起動したとき: リセットの原因(RCC_CSR)と、前回の HardFault の記録を読んで残す(記録は消す)。
//   シリアルに出し(FaultDiag_Print)、走行のログのイベントにも入れる(search_run.c の StartRun)。
// CCMRAM(.ccmram)は起動の処理(startup_stm32f405rgtx.s)でゼロにもコピーもされないので、リセットしても値が残る。
// 固まった(無限ループ・割り込みが止まらない)場合は HardFault にならないので、これでは分からない。

// RCC_CSR のリセットの原因のフラグ(CSR の bit25〜31 を右へ 24 ずらした値)
#define FAULT_RESET_BOR   (1u << 1) // 電源電圧の低下(ブラウンアウト)
#define FAULT_RESET_PIN   (1u << 2) // NRST ピン(どのリセットでも立つ)
#define FAULT_RESET_POR   (1u << 3) // 電源投入
#define FAULT_RESET_SFT   (1u << 4) // ソフトウェア(HardFault の後のリセットもこれ)
#define FAULT_RESET_IWDG  (1u << 5) // 独立ウォッチドッグ
#define FAULT_RESET_WWDG  (1u << 6) // ウィンドウウォッチドッグ
#define FAULT_RESET_LPWR  (1u << 7) // 低電力モード

typedef struct {
    uint32_t reset_flags; // FAULT_RESET_* の組み合わせ
    bool had_fault;       // このリセットの前に HardFault があった(以下が有効)
    uint32_t pc;          // 止まった命令のアドレス(.list / .map で関数を探す)
    uint32_t lr;          // 止まった関数を呼んだ所の近く
    uint32_t psr;
    uint32_t cfsr;        // 原因の詳細(bit0〜7 MemManage、bit8〜15 BusFault、bit16〜31 UsageFault)
    uint32_t hfsr;
    uint32_t mmfar;       // MemManage のアドレス(CFSR の MMARVALID のとき)
    uint32_t bfar;        // BusFault のアドレス(CFSR の BFARVALID のとき)
    uint32_t sp;          // 止まったときのスタックの位置
} FaultDiagInfo;

// 起動したら、HAL_Init の後なるべく早く1回呼ぶ。リセットの原因と前回の記録を読み、両方を消す。
void FaultDiag_ReadAtBoot(void);

// 起動時に読んだ内容
const FaultDiagInfo *FaultDiag_GetBootInfo(void);

// 起動時に読んだ内容をシリアルに出す(UART の初期化の後)。HardFault の後なら LED でも知らせる(約2秒)。
void FaultDiag_Print(void);

// stm32f4xx_it.c の HardFault_Handler から、アセンブラで呼ぶ(戻らない)。
// sp: 呼んだときの SP、exc_return: 例外に入ったときの LR(EXC_RETURN)。
void FaultDiag_HardFault(uint32_t *sp, uint32_t exc_return) __attribute__((noreturn));

#endif
