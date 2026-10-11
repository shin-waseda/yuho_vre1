# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

### 役割分担
- ファイルの編集は、ユーザーが明示的に依頼・許可した内容と範囲に限る
- 許可された範囲の外に問題や改善点を見つけた場合は、編集せずに報告だけする
- ファイルの読み取りや、質問・相談への回答は自由に行ってよい
- `yuho.ioc` の編集とコード再生成はユーザーが CubeIDE で行う

### ブランチ(git)
- ブランチを作るのは Claude の担当．新しいまとまった作業(新しい機能，大きな調整，不具合の修正など)を始めるとき，最初にファイルを変える前に作り，名前と元のブランチをユーザーに伝える
- 元は GitHub の既定のブランチ `master` の最新(`main` ではない)．まだ `master` に入っていない作業の続きなら，そのブランチから作る
- 名前は `feature/内容`(機能)，`fix/内容`(不具合の修正)．内容は英語の小文字とハイフン(例: `feature/front-wall-window`)
- 小さな続きの作業(調整値を1つ変える，開発日記の追記など)は今のブランチのまま行う
- `master` へのマージ(`--no-ff`)と `master` へのプッシュは，ユーザーに頼まれてから行う．マージやプッシュの前に `git ls-remote --symref origin HEAD` で既定のブランチを確かめる
- ブランチを消すときは，先にユーザーに確認する
- `.settings/stm32cubeide.project.prefs` は CubeIDE が環境ごとに書き換えるのでコミットしない

### ビルドと書き込み
- ビルドと書き込みはユーザーが STM32CubeIDE で行う
- ヘッドレスビルドを含め、ビルド・書き込みのコマンドは実行しない
- ビルドエラーはユーザーが貼り付けるので、それを元に回答する

### 応答の方針
- 返答は日本語で行う
- 確認していない事柄(レジスタ値、HAL の仕様、タイミングなど)を断定しない
  不確かな場合はその旨を明記する
- 仕様や意図が曖昧な場合は、推測で進めず質問する

## Project overview

`yuho` is the firmware for a micromouse (maze-solving) robot, running on an STM32F405RGT6 (Cortex-M4, 168 MHz). It is an STM32CubeIDE project generated from `yuho.ioc` via STM32CubeMX, with HAL drivers vendored under `Drivers/`.

## Build

This is an Eclipse CDT / STM32CubeIDE managed-build project (not a plain Makefile project you hand-write). STM32CubeIDE regenerates `Debug/makefile`, `Debug/objects.mk`, and `Debug/sources.mk` from the `.cproject` build config on each build.

- **Preferred**: open the workspace in STM32CubeIDE and build normally (Project > Build), or flash/debug via the IDE's ST-Link integration.
- **Headless build** (no GUI), from the STM32CubeIDE install dir:
  ```
  stm32cubeide -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild -build yuho/Debug -data <path-to-workspace>
  ```
- Build output (`yuho.elf`, `yuho.bin`) lands in `Debug/`. `Debug/**` is gitignored except for the `.bin`/`.elf` artifacts, which are intentionally committed.
- There is no test suite, linter, or CI config in this repo — verification is done on real hardware.

## Hardware configuration (`yuho.ioc`)

Peripheral/pin assignments are defined in `yuho.ioc` and must stay in sync with it — if you change pin usage or peripheral config, edit the `.ioc` and regenerate code via STM32CubeMX/CubeIDE rather than hand-editing the HAL init in `Core/Src/*.c` out of sync with it.

- **ADC1** (5-channel scan): wall IR sensors — `Sensor_L` (PA0), `Sensor_FR` (PA1), `Sensor_R` (PA2), `Sensor_FL` (PA3), `Vol_Check` battery voltage (PC0). Separate GPIO outputs `IR_L/IR_R/IR_FL/IR_FR` (PH1/PC15/PC14/PH0) gate the IR emitters, so a sensor read is an emitter-on/ADC-sample/emitter-off sequence (see the `*_on`/`*_off` globals in `main.c`).
- **TIM2**: motor PWM — CH1 = `Motor_L_PWM` (PA5), CH4 = `Motor_R_PWM` (PB11), period 4199 (`PWM_MAX`). Direction pins: `Motor_L_CW`/`Motor_L_CCW` (PC5/PC4), `Motor_R_CW`/`Motor_R_CCW` (PB1/PB10). `Motor_STBY` (PB0) must be set to enable the driver.
- **TIM3**: CH1 = `FAN` (PB4), CH2 = `Buzzer` (PB5).
- **TIM4 / TIM8**: quadrature encoder inputs (`ENC_L_A/B` on PB6/PB7, `ENC_R_A/B` on PC6/PC7), free-running 16-bit counters — read via delta-from-last-read, not an absolute position (see `Encoder_GetDeltaL/R`).
- **TIM6**: periodic base timer running as an interrupt (`HAL_TIM_Base_Start_IT`), intended as the main control-loop tick.
- **SPI2**: full-duplex master (PB13/14/15) with manual `CS` (PA4) — used for the ICM gyro.
- **SDIO**: 4-bit SD card bus (PC8-12, PD2) for logging; init call is currently commented out in `main.c`.
- **USART1**: async, used for `printf`/debug logging — `__io_putchar` in `main.c` routes stdio to it.
- 6 onboard LEDs (`LED_1`..`LED_6`) plus a 74HC595 shift register (`SER_595`/`Latch_595`/`SCLK_595`) for additional output, and `Push_IN_1` as a user button input.

## Code architecture

The codebase is mid-refactor from a flat STM32CubeMX layout into a layered one. Two things coexist right now:

1. **CubeMX-generated peripheral layer** (`Core/Src/*.c` + `Core/Inc/*.h` at the top level, e.g. `adc.c`, `tim.c`, `gpio.c`, `usart.c`, `stm32f4xx_it.c`) — standard HAL init code, regenerated from `yuho.ioc`. Treat `USER CODE BEGIN/END` markers as the only safe place to hand-edit these files, since CubeMX regeneration overwrites everything else.

2. **A new three-layer application structure** under `Core/Inc/` (headers only so far — most have no `.c` implementation yet and some don't even compile, e.g. `#include "logic/maze"` naming a directory):
   - **`interface/`** — thin hardware-facing wrappers around the HAL (`encoder.h`, `motor.h`, `gyro.h`, `sensor.h`, `led.h`, `uart.h`, `sdcard.h`, `flash.h`, `timer.h`). `interface/encoder.h`/`interface/motor.h` are the only ones with real implementations currently, in `Core/Src/encoder.c` and `Core/Src/motor.c`.
   - **`logic/`** — hardware-independent algorithms: `wall_sense.h`, `command.h`, and subdirectories `control/` (PID, velocity profile, wall control), `maze/` (Dijkstra solver, priority queue, wall map, step map), `state_estimation/` (odometry, kinematics, gyro fusion).
   - **`app/`** — top-level sequencing/state machine: `robot_state.h`, `drive_sequence.h`, `search_sequence.h`, `mode_ui.h`. These are meant to compose `interface/` and `logic/` into the actual search/run behavior.
   - `global.h` is the common include (currently just pulls in `main.h`); `params.h` is meant to hold tunable constants but is currently empty.

   When implementing new logic, follow this layering: `app/` depends on `logic/` and `interface/`; `logic/` should stay hardware-agnostic (no direct HAL calls) and depend only on `interface/` for I/O; `interface/` is the only layer allowed to touch HAL/CMSIS APIs directly.

`main.c`'s `while(1)` loop is currently ad hoc hardware bring-up/test code (spinning motors, dumping encoder and sensor readings over UART) rather than using the `app/` sequencing layer — expect this to be replaced as the layered architecture gets filled in.

Variable names, comments, and `printf` strings mix Japanese and English; follow the existing convention in whichever file you're editing rather than converting wholesale.
