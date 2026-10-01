# yuho_ver1 アーキテクチャ設計 議論まとめ

Claudeとの対話で決めた、yuho_ver1のディレクトリ構成と設計方針の記録。
実装時はこのルールに従うこと。

## 背景

過去機体(BlueEyes_Final)のコードをレビューした結果、以下の問題が見つかった。

- グローバル変数(`sensor_values`, `current_speed`, `MF`フラグレジスタ等)への
  依存が強く、ロジックとハードウェア操作が密結合していた
- ISR内でセンサー取得だけでなく壁制御則の計算まで行っていた
- `drive_trapezoid`等がbusy-waitでグローバル状態をポーリングする設計で、
  テストやシミュレーションに乗せにくい
- 壁のビットマスク等、機体依存の情報がロジック関数内にハードコードされていた

Simulink/MATLABでのシミュレーション環境構築・自動チューニングを将来やりたい、
という目標を踏まえ、「ロジック部分を純粋関数として分離する」ことを最優先に
アーキテクチャを再設計した。

## 層の定義と依存ルール

4層構成。依存の向きは以下の図の通り。

```
        params.h
       ↗   ↑   ↖
interface  global  logic
       ↖   ↓   ↗
          app
```

| 層 | 役割 | 持ってよいもの | 持ってはいけないもの |
|---|---|---|---|
| **interface** | ハードウェア直結 | HAL呼び出し、GPIO/ADC/SPI/SDIO直接操作 | 判断ロジック、閾値比較 |
| **logic** | 計算アルゴリズム | 入力構造体→出力構造体を返す純粋関数 | グローバル変数の読み書き、HAL_Delay、ハード型への依存 |
| **app** | シーケンス制御 | 状態遷移、busy-wait、logicとinterfaceの仲介 | アルゴリズムの中身そのもの |
| **global** | 共有基盤 | 型定義、ISR受け渡し用のvolatile変数 | 実処理ロジック |

判断基準:
- **A軸(機体依存かどうか)**: ハードウェアが変われば書き換わるものはinterface
- **B軸(副作用の有無)**: これを主軸にする。副作用なしの計算はすべてlogicに
  寄せる(シミュレーション環境への移植を見据えて)
- 「値」(閾値・ゲイン・寸法等の具体的な数値)はparams.h、「計算式」
  (アルゴリズムそのもの)はlogicに置く。例: PID制御式はlogic、
  Kp/Ki/Kdの値はparams.h

### 原則

- `interface`と`logic`は互いをincludeしない。`app`だけが両方を仲介する
- `logic`配下のファイル同士はincludeし合ってよい
- `interface`/`logic`は`global.h`と`params.h`のみ参照可能

### 唯一の例外: ISRコールバック

`HAL_TIM_PeriodElapsedCallback`等、HALが名前を決め打ちするコールバック関数は
`interface`に物理的に置くが、中身の処理(センサー取得・制御計算)は一切書かず、
`app`の関数を1回呼ぶだけに留める。これは「interfaceからappを直接呼んでよい」
という唯一の例外として明文化する(WMMCさんの先行コードも同じ割り切りだった
ことを確認済み)。

```c
// interface/timer.c (イメージ)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        app_control_tick();  // ← appの関数を呼ぶだけ。計算はしない
    }
}
```

※ `app_control_tick`相当の関数をどのapp配下ファイルに置くかは未確定
  (drive_sequence.hに含めるか、専用ファイルを新設するか)。

## 確定したディレクトリ構成

```
Core/Inc/
├── global.h                      型定義 + ISR受け渡し変数
├── params.h                      機体固有パラメータ(閾値・寸法・ゲイン等)
│
├── interface/
│   ├── motor.h
│   ├── sensor.h
│   ├── gyro.h                    ICM-42605, SPI2
│   ├── encoder.h                 TIM4/TIM8
│   ├── led.h
│   ├── flash.h                   内部フラッシュ
│   ├── sdcard.h                  SDIO
│   ├── uart.h                    USART1
│   └── timer.h                   TIM7待機、TIM6周期割り込み(ISR本体もここ)
│
├── logic/
│   ├── command.h                 経路コマンドのenum等(中身は未確定)
│   ├── wall_sense.h              壁の有無判定(周囲の判定)
│   ├── state_estimation/         「機体が今どう動いているか」の推定
│   │   ├── odometry.h             エンコーダ→速度・位置
│   │   ├── gyro_fusion.h          ジャイロ→角速度・姿勢
│   │   └── kinematics.h           機体速度⇔左右輪速度の変換
│   ├── control/                  制御則
│   │   ├── wall_control.h         壁追従補正量
│   │   ├── velocity_pid.h
│   │   ├── position_pid.h
│   │   └── velocity_profile.h     台形加減速の目標速度生成(旧trajectory、
│   │                               名前の見直し含め未確定)
│   └── maze/                     迷路探索アルゴリズム
│       ├── wall_map.h             壁情報の記録
│       ├── step_map.h             足立法の歩数マップ・経路生成
│       ├── dijkstra.h
│       └── priority_queue.h
│
└── app/
    ├── robot_state.h              ロボット全体の状態(app所有)
    ├── drive_sequence.h           走行シーケンス(busy-wait含む)
    ├── search_sequence.h          探索シーケンス
    └── mode_ui.h                  モード選択、LED表示との配線
```

Core/Srcも同じ構成でミラーする。

## PID制御に関する補足

速度PID・位置PIDは「機体依存に見えるが、計算式自体は機体非依存」という
整理をした。ゲイン値(Kp/Ki/Kd)はparams.hに置き、計算式自体はlogic/control/
に置く。モーターへの出力(PWM書き込み)はinterfaceの役割。

## 未決定・保留中の論点

- `command.h`の具体的な列挙子設計
  - コマンドの粒度(1区画ごとか、BlueEyesの`0x8N`のような複数区画圧縮か)
  - スラローム/通常ターンの区別をRouteCommand自体に持たせるか、別軸の
    情報として分離するか
- `velocity_profile.h`という名前が実態に合っているか(要再検討)
- ISRから呼ぶ1kHz制御ループの実処理をどのapp配下ファイルに置くか
- app層が`robot_state`/`drive_sequence`/`search_sequence`/`mode_ui`の
  4ファイルで十分か
- SDカード(ログ記録等)の用途の詳細設計

## 参考にした既存コード

- BlueEyes_Final(過去機体): 層構造はあったが境界が守られておらず、
  反面教師として多くの示唆を得た
- WMMCさん設計(NucleoMouse2026): motor/drive/searchの3層分離が参考になった。
  ただしISRからapp相当の関数を直接呼んでおり、今回の例外ルールの前例とした