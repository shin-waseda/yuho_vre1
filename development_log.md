# yuho 開発記録

マイクロマウス yuho(STM32F405RGT6)のファームウェアと PC ツールの開発記録．
立ち上げ(2026-06)から 2026-10-08 時点までの，実装の目的・経過・結果・設計の考え方・調整のやり方をまとめる．

- コミット済みの内容は `git log` のハッシュを付けた．
- 2026-10-08 時点で **未コミット** の内容(bbfbcf7 より後)は「未コミット」と書いた．
- 「未確認」と書いたものは，実機やログでまだ確かめていない．

---

## 目次

1. [機体とプロジェクトの概要](#1-機体とプロジェクトの概要)
2. [開発の進め方](#2-開発の進め方)
3. [設計の考え方](#3-設計の考え方)
4. [実装の解説(コードで読む)](#4-実装の解説コードで読む)
5. [開発の経過(時系列)](#5-開発の経過時系列)
6. [調整のやり方(手順集)](#6-調整のやり方手順集)
7. [ログの仕組みと使い方](#7-ログの仕組みと使い方)
8. [PC ツール](#8-pc-ツール)
9. [つまずいたことと教訓](#9-つまずいたことと教訓)
10. [今のパラメータ(2026-10-08)](#10-今のパラメータ2026-10-08)
11. [残っている課題](#11-残っている課題)
12. [開発日記(2026-10-08 以降)](#12-開発日記2026-10-08-以降)

---

## 1. 機体とプロジェクトの概要

| 項目 | 内容 |
|---|---|
| マイコン | STM32F405RGT6(Cortex-M4，168MHz) |
| 開発環境 | STM32CubeIDE 1.19.0(`yuho.ioc` から CubeMX でコード生成) |
| モーター | FAULHABER，エンコーダ IEH2-4096(4逓倍)，ギア 13:42 |
| タイヤ | 実測 φ23.35mm(走行に合わせた実効値 23.81mm) |
| トレッド | 設計 57.90mm(ジャイロに合わせた実効値 64.75mm) |
| ジャイロ | ICM-42688-P(SPI2，±2000dps) |
| 壁センサー | IR 4個(L，FL，FR，R)．L/R は斜め前向き．値は点灯 − 消灯の差 |
| 電池 | 2S LiPo(分圧 33k/20k で PC0) |
| 記録 | SD カード(SDIO 4bit + DMA + FatFs)，UART |
| 表示 | 直結 LED 6個 + 74HC595 のシフトレジスタの LED |
| 制御周期 | TIM6 の 1kHz 割り込み |

参考にした先行コード:
- **BlueEyes_Final**(過去機体，F303K8): 迷路の判断・探索の流れ(searchB，start_sequence，set_position)の元．
- **divergence_v3**(akiaki96): ログ機能の元．
- **akiaki96/maze_sim_py_c_v2**: 迷路シミュレータの構成の参考．

---

## 2. 開発の進め方

- **役割分担**: ビルド・書き込み・実機での走行はユーザーが行う．Claude はコードの編集(許可された範囲)，ログの解析，PC ツールの作成を行う．`.ioc` の編集とコード生成はユーザーが CubeIDE で行う．
- **ログで決める**: パラメータの多くは「走らせる → SD のログを取る → ログを見て直す」で決めた．経緯は `params.h` のコメントにも残している．
- **段階を踏む**: 低い速度で確かめてから上げる(例: 直進 300 → 800mm/s，探索 300 → 500mm/s)．
- **機体が使えない間は logic とシミュレータ**を進める(迷路の判断・最短経路は PC で先に確かめた)．

---

## 3. 設計の考え方

### 3.1 層の構成(詳しくは `architecture_discussion.md`)

```
Core/Inc, Core/Src
├── interface/   ハードに直結(HAL を呼んでよいのはここだけ)
├── logic/       計算(入力 → 出力の純粋な関数．グローバル変数・HAL_Delay を使わない)
│   ├── control/          PID，台形，壁の制御，スラロームの形
│   ├── maze/             壁の地図，Dijkstra，足立法，探索プランナー，最短の指令
│   └── state_estimation/ 運動学，オドメトリ
├── app/         手順・状態遷移(logic と interface をつなぐ)
├── global.h     共有の型
└── params.h     数値(ゲイン・寸法・しきい値)．式は logic に置く
```

- 判断の基準は「**副作用があるか**」．副作用のない計算はすべて logic に寄せる．
  そうすると，同じ C のソースを PC でコンパイルしてシミュレーションできる(`tools/maze_sim` がそうしている)．
- `interface` と `logic` は互いを include しない．`app` だけが両方を使う．
- 例外は HAL のコールバック(`HAL_TIM_PeriodElapsedCallback`)だけ．中身は書かず，`app` の関数を1回呼ぶ．

### 3.2 制御の考え方

- **カスケード制御**: 外側に位置(並進)と角度・角速度，内側に車輪速度の PI + FF を置く．
  ```
  v_cmd = v_ref + POSITION_KP × (s_ref − s)
  ω_cmd = ω_ref + ANGLE_KP × (θ_ref − θ_gyro) + ANGULAR_KP × (ω_ref − ω_gyro)
  車輪の目標 = v_cmd ∓ ω_cmd × TREAD/2
  電圧 = FF(目標だけから) + PI(車輪の目標 − 実速度)
  duty = 電圧 / 電池電圧
  ```
- **出力は電圧 [V]**: 電池で割って duty にするので，電池が減ってもゲインの効きが変わらない．
- **2自由度制御**: FF は目標(プロファイル)だけから計算し，実測を通さない．外側の補正(位置・角度)も FF には入れない．
- **常に効かせる**: 止まっている間も向きと位置を保つ．
- **閾値・不感帯で隠さない**: 振動(リミットサイクル)が出たら，閾値で制御を切るのではなくゲインで対処する(ユーザーの方針)．
- **目標は積分で作り，リセットしない**: θ_ref，s_ref は制御を有効にした時点を 0 にし，プロファイルを積分して作る．旋回ごとのずれが積み重ならない．
- **調整の目安は I 項**: FF が合っていれば，定常の I 項はほぼ 0 になる．I 項の符号と大きさから FF を直す．

### 3.3 迷路と走行の考え方

- **BlueEyes と同じ流れ**: 最初の半区画，境界で壁を読む，境界で経路を計算し直す，尻当て(set_position)．
- **地図のビット配置**: 1区画1バイトで BlueEyes と同じ．
  - 下位 4bit は探索用で，未知の壁は「ない」とみなす．
  - 上位 4bit は最短走行用で，未知の壁は「ある」とみなす．
  - 各 4bit は 北 0x8，東 0x4，南 0x2，西 0x1．
- **指令(アクションコマンド)を介す**: logic は「次にどう動くか」だけを返し，どう動くか(超信地かスラロームか)は実行側が決める．これでシミュレータと実機が同じ判断部分を使える．
- **最短経路は走行時間で選ぶ**(コストではなく)．
- **シミュレータでは軌道を作り込まない**: シミュレータの目的は経路の選び方の評価．軌道は実機側で作る(ユーザーの方針)．

### 3.4 ログの考え方

- **何でも記録する**: 制御の内部値・センサー・電池に加えて，イベント(モード，ボタン，指令，壁の判定，補正)も残す．
- **走りを止めない**: 長い走行は走りながら SD へ流す(DMA で待たない)．
- **マイコンは生のバイナリを書くだけ**: CSV への変換やグラフは PC がやる．

### 3.5 UI の考え方

- モードは右タイヤを回して選び，ボタンで決める．階層は RUN / TEST / SD．番号は 1 から(n 番 = LED n, n+1)．
- 走り出しは手かざし(FL のセンサー)．機体に触れずに始められるので，置いた位置がずれない．
- エラーは直結 LED の場所で知らせる．電圧低下は右後ろ，SD の失敗は左後ろ．

---

## 4. 実装の解説(コードで読む)

この章では，実際のコードを引用しながら「何をしているか」と「なぜそう作ったか」を説明する．
引用は要点だけを抜き出したもので，`...` は省略を表す．ファイルの場所は見出しに書いた．
数値は 2026-10-08 時点の `params.h` の値．

### 4.1 1ms の割り込みの流れ(`interface/timer.c`)

制御のすべては TIM6 の 1kHz の割り込みの中で動く．HAL が名前を決めているコールバックは interface に置き，中身は書かずに app の関数を呼ぶだけにしている(3.1 の唯一の例外)．

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance != TIM6) return;

  Sensor_ReadAll();   // 1. 壁センサーと電池電圧を読む
  App_ControlTick();  // 2. 制御を1周期進める(目標 → 補正 → PWM)
  Logger_Sample();    // 3. このtickで確定した値を記録する
}
```

順番に意味がある．センサーを読んでから制御するので，制御は「今の tick の」センサー値を使う．制御の後に記録するので，ログの1行は「その tick の入力・目標・出力」がそろった状態になる．

メイン(`while` のループ，探索の手順など)は割り込みの合間に動き，目標を渡したり，終わるのを待ったりするだけ．計算の重い経路探索(`SearchPlanner_Step`，20〜25ms)もメイン側で動くので，制御は止まらない．

### 4.2 壁センサーの読み方(`interface/sensor.c`)

IR の LED を消した値と点けた値の**差**を使う．外の光(蛍光灯・日光)の影響を打ち消すため．

```c
// --- 1. 全LED消灯でOFF値を取得 ---
...
r_off  = adc_next();  fr_off = adc_next();  fl_off = adc_next();  l_off = adc_next();
vabt   = adc_next();

// --- 2. R だけ点けて読む ---
HAL_GPIO_WritePin(IR_R_GPIO_Port, IR_R_Pin, GPIO_PIN_SET);
tim6_wait_us(IR_SETTLE_US);          // 50us 待って光を安定させる
r_on = adc_next();
HAL_GPIO_WritePin(IR_R_GPIO_Port, IR_R_Pin, GPIO_PIN_RESET);
... (FR, FL, L も同じ)

// --- 6. 差分(アンダーフローしないように) ---
ad_r = (r_on > r_off) ? (r_on - r_off) : 0;
```

- 4つの LED を**1つずつ**点ける．同時に点けると，隣のセンサーの光を拾ってしまう．
- 待ち時間 50us は TIM6 のカウンタ(1MHz)で数える．割り込みの中なので `HAL_Delay` は使えない．
- ADC は「次の rank を1つだけ変換する」設定(Discontinuous)にしてあり，`adc_next()` を呼ぶたびに R → FR → FL → L → 電池 の順に進む．

この値(`ad_l` など)は mm に直さず，そのまま使う(5.14 の方針)．

### 4.3 エンコーダとオドメトリ(`interface/encoder.c`，`params.h`)

エンコーダのカウンタ(TIM4/TIM8)は 16bit で回り続ける．絶対値ではなく，**前回からの差**を使う．

```c
int16_t Encoder_GetDeltaL(void) {
    uint16_t now = __HAL_TIM_GET_COUNTER(&htim4);
    int16_t delta = (int16_t)(now - s_last_l); // 16bitのラップアラウンドを利用した差分計算
    s_last_l = now;
    return delta;
}
```

`uint16_t` どうしの引き算を `int16_t` にすると，65535 → 0 と回り込んでも正しい差になる(1ms で ±32767 パルスを超えなければよい)．

パルスから距離への換算:

```c
#define ENCODER_PULSES_PER_REV (ENCODER_PPR * ENCODER_MULTIPLIER * GEAR_SPUR_T / GEAR_PINION_T)
//                            = 4096 × 4 × 42 / 13 ≈ 52935 [パルス/タイヤ1回転]
#define MM_PER_PULSE ((3.14159265f * WHEEL_DIAMETER_MM) / ENCODER_PULSES_PER_REV)
//                    ≈ 0.00141 mm/パルス
```

エンコーダはモーターの軸に付いているので，ギア比を掛ける．`WHEEL_DIAMETER_MM` はノギスの値ではなく，走らせた距離に合わせた実効値(6.3)．

### 4.4 運動学: 車輪の速さ ↔ 機体の速さ(`logic/state_estimation/kinematics.c`)

```c
WheelVelocity Kinematics_RobotToWheel(RobotVelocity robot) {
    float turn_term = (robot.angular_rad_s * TREAD_WIDTH_MM) / 2.0f;
    WheelVelocity w;
    w.left_mm_s  = robot.linear_mm_s - turn_term;
    w.right_mm_s = robot.linear_mm_s + turn_term;
    return w;
}
```

左右の車輪の速さ v_L, v_R と，機体の並進 v・回転 ω の関係は

- v = (v_L + v_R)/2
- ω = (v_R − v_L)/T(T はトレッド，反時計回りが +)

`TREAD_WIDTH_MM` も設計値(57.90)ではなく，ジャイロで測った回り方に合わせた実効値(64.75)．タイヤの接地点のずれや横滑りを含む．

### 4.5 PID のコア(`logic/control/pid.c`)

```c
float PID_Update(PID_t *pid, float error, float dt, float output_min, float output_max) {
    float derivative = (dt > 0.0f) ? (error - pid->prev_error) / dt : 0.0f;
    pid->prev_error = error;

    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    if (output <= output_max && output >= output_min) {   // 出力が飽和していないときだけ
        pid->integral += error * dt;                        // 積分を進める
        output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    }

    if (output > output_max) output = output_max;
    if (output < output_min) output = output_min;
    return output;
}
```

式は u = Kp·e + Ki·∫e dt + Kd·de/dt．

- **積分の止め方(アンチワインドアップ)**: 出力が上限・下限に張り付いている間は積分を進めない．張り付いている間も積分を続けると，I 項が溜まりすぎ，目標に届いた後に大きく行き過ぎる．
- **微分は誤差の微分**．今は `VELOCITY_KD = 0` で，微分は使っていない(P と I で足りている)．
- この関数は**状態(`integral`，`prev_error`)を構造体に持つ純粋な計算**で，HAL もグローバル変数も使わない．だから logic 層にあり，PC でもテストできる．

車輪速度の PI は，左右に1つずつこの `PID_t` を持つ(`logic/control/velocity_pid.c`)．

### 4.6 車輪速度のループと FF(`app/control_loop.c` の `App_ControlTick`)

制御の出力はモーターに**かける電圧 [V]** で，最後に電池電圧で割って PWM にする．

```c
// 2自由度制御: FFは目標(速度・加速度)だけから、PIDは偏差から。単位はどちらも電圧[V]。
float ff_l = VELOCITY_FF_FRIC_L * SignOf(target_wheel_ff.left_mm_s)    // 摩擦(速さによらず一定)
           + VELOCITY_FF_GAIN_L * target_wheel_ff.left_mm_s             // 速さに比例(逆起電力など)
           + VELOCITY_FF_ACC_L  * target_wheel_acc.left_mm_s;           // 加速に要る分
...
out = VelocityPID_Update(&s_vpid, target_wheel, s_actual, CONTROL_DT_S);   // 補正(PI)
s_pwm.left = VoltageToPWM(ff_l + out.left_mm_s, vbat);
```

```c
static int16_t VoltageToPWM(float volt, float vbat) {
    float pwm = volt / vbat * (float)PWM_MAX;    // duty = 電圧 / 電池電圧
    ...
}
```

**なぜ電圧で考えるか**: DC モーターの式はおおよそ「電圧 = 摩擦の分 + 速さに比例する分 + 加速に比例する分」になる．PWM の duty で考えると，電池が 8.2V から 7.4V に下がるだけで同じ duty でも出る電圧が 10% 変わり，ゲインの効き方がずれる．電圧で考えて最後に電池電圧で割れば，電池の減り方によらず同じ効き方になる．

**なぜ FF を目標だけから作るか(2自由度制御)**: FF は「この目標ならこれくらいの電圧が要る」という見込みで，ほとんどの電圧を FF が出し，PI は見込みとのずれだけを直す．FF に実測や外側の補正を混ぜると，止まっているとき(目標 0，補正がほぼ 0)に補正の小さな揺れで `SignOf` の符号が入れ替わり，摩擦の電圧(±0.3V)がバタバタと反転してしまう．

```c
// 摩擦FF用の符号。目標0のときは0(止まっている所に摩擦ぶんの電圧をかけない)。
static float SignOf(float v) {
    if (v > 0.0f) return 1.0f;
    if (v < 0.0f) return -1.0f;
    return 0.0f;
}
```

**超信地旋回だけの摩擦 FF**: その場で回るときはタイヤが横にこすれるので，直進用の式では 0.75〜0.84V 足りなかった．回る向きで要る電圧も違ったので，4つに分けている．

```c
if (s_target_mm_s == 0.0f && s_target_acc == 0.0f && s_target_omega_dps != 0.0f) {
    if (s_target_omega_dps > 0.0f) {   // 反時計回り: 右の車輪が前へ、左が後ろへ
        ff_l -= PIVOT_FF_FRIC_CCW_L;
        ff_r += PIVOT_FF_FRIC_CCW_R;
    } else { ... }
}
```

**FF が合っているかの見分け方**: ログの `i_l`，`i_r`(I 項の電圧)を見る．FF がぴったりなら，定常の I 項は 0 付近になる．調整のやり方は 6.1．

### 4.7 外側のループ: 角度と位置(`app/control_loop.c`)

車輪速度のループの外側に，機体の向き(ジャイロ)と進んだ距離(エンコーダ)のループがある．出力は「車輪速度の目標への上乗せ」．

```c
// ---- 位置のループ(外側、並進方向) ----
// v_cmd = v_ref + POSITION_KP×(s_ref − s)
pos_corr = POSITION_KP * (s_pos_ref_mm - s_dist_mm);           // 上限 ±200mm/s

// ---- 角度・角速度のループ(外側) ----
// ω_cmd = ω_ref + ANGLE_KP×(θ_ref − θ) + ANGULAR_KP×(ω_ref − ω)
ang_corr = ANGLE_KP * angle_err_deg * DEG_TO_RAD + ANGULAR_KP * (omega_ref - omega_meas);

RobotVelocity target_robot = {
    .linear_mm_s = s_target_mm_s + pos_corr,
    .angular_rad_s = omega_ref + ang_corr,
};
WheelVelocity target_wheel = Kinematics_RobotToWheel(target_robot);
```

**目標の向き・距離の作り方**: 目標の向き θ_ref と目標の距離 s_ref は，制御を有効にした時点を 0 とし，プロファイルの目標を毎 tick 積分して作る．

```c
if (control_active) {
    s_angle_axis_deg += s_target_omega_dps * CONTROL_DT_S;   // 目標の向き(迷路の軸)
    s_pos_ref_mm     += s_target_mm_s * CONTROL_DT_S;        // 目標の距離
    s_dist_mm += 0.5f * (s_actual.left_mm_s + s_actual.right_mm_s) * CONTROL_DT_S; // 進んだ距離
    ...
} else {
    s_angle_axis_deg = s_gyro_angle_deg;   // 無効の間は今の向きに合わせておく
    s_pos_ref_mm = 0.0f;  s_dist_mm = 0.0f;
}
```

止まっても目標をリセットしないので，90° 回るたびに 0.5° ずつずれる，のようなずれが積み重ならない(旋回の終わりのずれは，次の動きの間に直される)．

**角度のずれが縮む速さ**: 内側(車輪速度)が十分速く，ω = ω_cmd になるとする．角度の誤差を e = θ_ref − θ とすると de/dt = ω_ref − ω なので，

```
ω_ref − ω = −K_θ·e − K_ω·(ω_ref − ω)
→ (1 + K_ω)(ω_ref − ω) = −K_θ·e
→ de/dt = −K_θ / (1 + K_ω) · e
```

誤差は時定数 (1 + K_ω)/K_θ = (1 + 0.5)/25 = 0.06 秒で縮む．同じように，位置の誤差は時定数 1/K_p = 1/10 = 0.1 秒で縮む．

**止まっている間も効かせる**: 止まったら制御を切る(閾値で I 項を捨てるなど)作り方もあるが，yuho では切らない．振動が出たらゲインで直す(3.2)．PARTY モードで床ごと回しても同じ向き・位置に戻るのは，これが常に効いているから．

**尻当ての間は止める**: 壁に押し当てている間は，機体が壁にそろって回るのを打ち消さないよう，向きと位置の補正を止める(`s_wall_push`)．

### 4.8 目標の作り方: 台形プロファイル(`logic/control/velocity_profile.c`)

直進・超信地旋回・スラロームの角速度は，すべて同じ台形のコードで作る．超信地旋回とスラロームでは「距離 = 角度 [deg]，速さ = 角速度 [dps]」として使う．

```c
void VelocityProfile_Step(VelocityProfile *p, float dt) {
    float remaining = p->distance_mm - p->pos_mm;

    // 減速開始の判定: 今の速度からv_endまで減速するのに要る距離に、1tick先の位置が入ったら減速に入る
    if (!p->decelerating) {
        float decel_dist = (p->v * p->v - p->v_end * p->v_end) / (2.0f * p->accel);
        if (remaining - p->v * dt <= decel_dist) p->decelerating = true;
    }

    float a_cmd;
    if (p->decelerating) {
        // 残りの距離でちょうどv_endになる減速度を毎tick計算し直す
        a_cmd = -(p->v * p->v - p->v_end * p->v_end) / (2.0f * remaining);
        if (a_cmd < -PROFILE_DECEL_MARGIN * p->accel) a_cmd = -PROFILE_DECEL_MARGIN * p->accel;
    } else if (p->v < p->v_max) {
        a_cmd = p->accel;      // 加速
    } else {
        a_cmd = 0.0f;          // 等速
    }
    float v_next = p->v + a_cmd * dt;
    ...
    p->pos_mm += 0.5f * (p->v + v_next) * dt;   // 台形の面積で進める
```

- **減速度を毎 tick 計算し直す理由**: 減速度を固定にすると，1ms 刻みの誤差で，終点に速度が残ったまま着いたり，手前で止まったりする．「残りの距離 d で v から v_end にする」減速度 a = (v² − v_end²)/(2d) を毎回計算すれば，ぴったり終点で v_end になる．上限(指定の 1.5 倍)は，計算が暴れたときの保険．
- **`v_start` で直進をつなぐ**: 走りながら次の直進を始めると，今の速さから始まる(`UpdateProfile` で `v_start = s_target_mm_s`)．探索で区画ごとに止まらずに走れるのはこのため．

### 4.9 メインと割り込みの受け渡し(`app/control_loop.c`)

メインと割り込みは同じ変数を触るので，壊れないように順番を決めている．

**動きの指令**: パラメータを書いてから，最後にフラグを立てる．割り込みは次の tick でフラグを見て取り込む．

```c
void App_StartStraight(float distance_mm, float v_max, float v_end, float accel) {
    s_pending.type = MOTION_STRAIGHT;
    s_pending.distance = fabsf(distance_mm);
    ...
    s_motion_done = false;
    s_start_pending = true; // 最後に立てる
}
```

フラグを先に立てると，パラメータを書いている途中で割り込みが取り込んでしまう可能性がある．

**割り込み側の値を読む**: 壁切れの位置などは，割り込みが「最後に通し番号を進める」．メインは番号が読む前と後で同じかを確かめ，違えば読み直す．

```c
uint32_t App_GetWallEdge(uint8_t *side, float *pos_ref_mm) {
    uint32_t seq;
    do { // ISR が途中で書き換えたら読み直す
        seq = s_edge_seq;
        if (side != NULL) *side = s_edge_side;
        if (pos_ref_mm != NULL) *pos_ref_mm = s_edge_pos_ref;
    } while (seq != s_edge_seq);
    return seq;
}
```

**割り込み側の変数を 0 にしたいとき**: ジャイロの積算角はメインが直接 0 にせず，「0 にしてほしい」というフラグを立て，割り込みが 0 にしてからフラグを下ろす．割り込みが足し込んでいる途中で 0 にされる，ということが起きない．

### 4.10 壁の制御(`logic/control/wall_control.c`)

```c
float err_l = (float)v.l - (float)WALL_REF_L; // 正: 左の壁に近い
float err_r = (float)v.r - (float)WALL_REF_R; // 正: 右の壁に近い
float err;
if (use_l && use_r)  err = err_l - err_r;      // 両方: 左右の差
else if (use_l)      err = 2.0f * err_l;       // 片方だけ: その側の差の2倍
else if (use_r)      err = -2.0f * err_r;
else                 return 0.0f;              // 壁なし: オフセット 0(軸の向きへ戻る)

float speed = (speed_mm_s > WALL_KP_MIN_V_MM_S) ? speed_mm_s : WALL_KP_MIN_V_MM_S;
float kp = WALL_KP_DEG * (WALL_KP_REF_V_MM_S / speed);   // 速いほど弱く
float offset = -kp * err;                                // 左に寄っていれば右へ向ける
```

制御の側では，目標の向きを「迷路の軸 + オフセット」にする．

```c
s_angle_ref_deg = s_angle_axis_deg + s_wall_offset_deg;
```

**なぜオフセットの形か**: 最初は「誤差に比例して向きを回す速さ」(向きを積分する形)にしていた．すると大きく寄ったときに向きが 8.7° 回ったまま残り，曲がった後に斜めのまま壁にぶつかった(5.15 の記録の経緯)．オフセットの形なら，真ん中に戻れば(誤差 0)オフセットも 0 になり，向きは迷路の軸に戻る．

**なぜゲインを 1/v にするか**: 横に戻る速さは「速さ × sin(オフセット)」なので，同じオフセットでも速いほど速く戻る．ゲインを 1/v にすると，真ん中に戻るまでの時間が速さによらず同じになる．

**壁の切れ目では使わない**: 2mm 進むごとに値を比べ，15 を超えて変わった側は 40mm の間使わない．切れ目や柱の横では値が急に変わり，「壁が遠い」と区別がつかないため．

```c
if (dist_mm - s->last_dist_mm >= WALL_EDGE_STEP_MM) {
    int32_t diff = (int32_t)value - (int32_t)s->last_value;
    if (diff > WALL_EDGE_DIFF || diff < -WALL_EDGE_DIFF) {
        s->hold_until_mm = dist_mm + WALL_EDGE_HOLD_MM;
    }
    ...
}
return value > threshold && dist_mm >= s->hold_until_mm;
```

オフセットを変える速さには上限(90dps)を付けている．壁が切れたときなどに，目標の向きが跳ばないように．

### 4.11 壁切れ補正(`app/control_loop.c`，`app/search_run.c`)

横の壁が切れた瞬間は，機体が区画の境界を決まった距離(`WALL_EDGE_POS_MM` = 78mm，仮)だけ過ぎた所にいるはず．そこで距離の数え方のずれを直す．

割り込みで「壁あり → 壁なし」に変わった瞬間を見つける:

```c
if (value > th) {                       // 壁あり
    if (!t->above) { t->above = true; t->above_from = s_pos_ref_mm; }
    return;
}
if (t->above) {                         // 壁あり → 壁なし に変わった
    t->above = false;
    if (s_pos_ref_mm - t->above_from >= WALL_EDGE_MIN_WALL_MM) {   // 30mm 以上続いた壁だけ
        s_edge_side = side;
        s_edge_pos_ref = s_pos_ref_mm;
        s_edge_seq++; // 最後に進める(メインはこれが変わったら読む)
    }
}
```

メイン側で，一番近い境界からのずれを求めて，次の境界の位置をずらす:

```c
// 一番近い境界(boundary0 + 1区画 × k)を選ぶ
float k = floorf((pos - WALL_EDGE_POS_MM - ec->boundary0) / SECTION_MM + 0.5f);
float expected = ec->boundary0 + k * SECTION_MM + WALL_EDGE_POS_MM;
float c = pos - expected;
if (c > WALL_EDGE_WINDOW_MM || c < -WALL_EDGE_WINDOW_MM) return false; // 予想から外れすぎ(使わない)
if (c > WALL_EDGE_MAX_CORR_MM) c = WALL_EDGE_MAX_CORR_MM;               // 1回に直すのは ±20mm まで
...
*corr = c;   // 呼び出し側が、待つ位置(次の境界)を c だけずらす
```

- 柱の横で一瞬だけ跳ねた値に引っかからないよう，その前に 30mm 以上壁が続いたときだけ数える．
- 予想から ±30mm より外れた壁切れは，読み違いか別の壁とみなして使わない．
- 結果はイベント `EDGE_CORR`(expected，edge_pos)に残るので，`WALL_EDGE_POS_MM` はログを見て合わせる．

### 4.12 スラロームの形(`logic/control/slalom.c`)

スラロームは，並進の速さ v を保ったまま，角速度だけを台形で動かして曲がる．曲がる前後の直進(オフセット)の長さを，**機体と同じ台形を 1ms ずつ進めて**求める．

```c
SlalomShape Slalom_ComputeShape(const SlalomParams *p) {
    VelocityProfile prof;
    VelocityProfile_Start(&prof, p->angle_deg, 0.0f, p->omega_dps, 0.0f, p->alpha_dps2);
    float theta = 0.0f;
    while (!prof.done && ticks < 100000u) {
        VelocityProfile_Step(&prof, CONTROL_DT_S);   // ISR と同じ角速度の台形
        theta += prof.v * CONTROL_DT_S;              // その tick の向き
        float th = theta * DEG_TO_RAD;
        s.forward_mm += p->v_mm_s * cosf(th) * CONTROL_DT_S;   // 曲がり始めの向きへ進んだ距離
        s.side_mm    += p->v_mm_s * sinf(th) * CONTROL_DT_S;   // 曲がる側へ進んだ距離
        ...
    }
}

void Slalom_Turn90Offsets(const SlalomShape *s, float span_mm, float *pre_mm, float *post_mm) {
    *pre_mm = span_mm - s->forward_mm;    // 前のオフセット
    *post_mm = span_mm - s->side_mm;      // 後ろのオフセット
}
```

- 小回り 90° は span = 半区画(境界の真ん中 → 隣の境界の真ん中)．500mm/s・450dps・8000dps² で pre ≈ 12.3mm，post ≈ 11.4mm．
- 大回り 90° は span = 1区画(区画の中心 → 斜め隣の中心)．
- 大回り 180° は「横にちょうど 1区画移る」角速度を二分法で求める．
- **なぜ数式ではなく数値で積分するか**: 機体は 1ms ごとに台形を進めて曲がるので，同じ計算をすれば離散化の誤差まで一致する．PC の turn_sim も同じ計算をしていて，スリップ 0 なら出口のずれは 0.00mm になる．

**速さを変えても形を保つ**:

```c
void Slalom_ScaleToSpeed(SlalomParams *p, float v_mm_s) {
    float k = v_mm_s / p->v_mm_s;
    p->omega_dps *= k;        // 曲率 κ = ω / v を保つ
    p->alpha_dps2 *= k * k;   // 角速度の台形を「進んだ距離に対して」同じ形にする
    p->v_mm_s = v_mm_s;
}
```

### 4.13 探索の流れ(`app/search_run.c`，`logic/maze/search_planner.h`)

判断(どこへ行くか)は logic 層の `SearchPlanner`，動き(どう走るか)は app 層が受け持つ．その間は，壁の観測 `WallObservation` と指令 `Action` だけでやり取りする．

```c
while (1) {
    Action act = SearchPlanner_Step(&s_planner, obs);   // 今の区画の壁を渡して、次の指令をもらう
    Record(pos, heading, obs, sv, act, plan_ms);        // STEP イベントに残す
    switch ((ActionType)act.type) {
        case ACTION_FORWARD:
            ok = GoToNextBoundary(&dp, act.cells, &obs, &sv);    // 次の境界まで走り、壁を読む
            break;
        case ACTION_TURN_RIGHT:
        case ACTION_TURN_LEFT:
            if (s_search_slalom && !dp.at_center) ok = SlalomTurn(&dp, ...);   // 止まらずに曲がる
            else ...                                                            // 真ん中で超信地旋回
        ...
    }
}
```

- `SearchPlanner_Step` は PC のシミュレータ(`tools/maze_sim`)でも同じものが動く．シミュレータでは「壁を読む」を迷路のファイルから作り，「走る」を位置を進めるだけにしている．
- 壁は区画の境界で読む．境界に着いた時点で，センサーは入ろうとする区画の壁を見ている．
- 距離は「目標の距離の絶対値」(`dp->ref_mm`)で決める．経路の計算に時間がかかっても，その間に進んだ分が積み重ならない．

### 4.14 フェイルセーフ(`app/failsafe.c`)

```c
// 条件が成立している間カウントを進め、limit_msに達したらtrueを返す。
static bool Persist(uint16_t *counter_ms, bool cond, uint16_t limit_ms) {
    if (!cond) { *counter_ms = 0; return false; }
    if (*counter_ms < limit_ms) (*counter_ms)++;
    return *counter_ms >= limit_ms;
}
...
if (Persist(&s_low_voltage_ms, s_vbat_filtered < FAILSAFE_LOW_VOLTAGE_V, FAILSAFE_LOW_VOLTAGE_MS)) {
    FailSafe_Trip(FAILSAFE_LOW_VOLTAGE, s_vbat_filtered);
}
```

- 「続けて○ms 成り立ったら」発動する．加速した瞬間の電圧の落ち込みなど，一瞬のことでは止まらない．
- 一度発動したら，リセットまで戻らない(ラッチ)．割り込みが毎 tick モーターを止め直すので，メインが誤って有効にしても止まる．
- 原因は直結 LED で知らせる(電圧低下は右後ろ)．

### 4.15 SD カード(1): ふつうの書き方(`interface/sdcard.c`)

試験モードの保存や SD_DUMP では，FatFs(`f_open`，`f_write`，`f_close`)をそのまま使う．

```c
FRESULT res = f_open(&s_file, path, FA_CREATE_NEW | FA_WRITE);   // 新しい番号のファイルを作る
...
FRESULT res = f_write(&s_file, data, len, &written);
...
f_close(&s_file);
```

**ファイルの番号**: `dir/prefix_NNNN.bin` の NNNN を 1 から順に探し，`.csv` でも `.bin` でも，送った後の `sent/` の中でも使われていない番号にする．PC で `.bin` から `.csv` を作ったとき，昔の同じ番号の `.csv` とぶつからないようにするため．

**CubeMX が作ったコードの不具合を直す**: 生成された `sd_diskio.c` は，カードが前の書き込みを内部で処理している間(PROGRAMMING，数 ms)に次の操作を始めると，`FR_INVALID_OBJECT` で失敗する．生成ファイルは触らず(再生成で消えるため)，`bsp_driver_sd.c` の `__weak` 関数を同じ名前で定義し直して，カードが待機(TRANSFER)に戻るまで待つようにした．

```c
static bool WaitCardReady(uint32_t timeout_ms) {
    uint32_t t0 = HAL_GetTick();
    while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER) {
        if (HAL_GetTick() - t0 >= timeout_ms) return false;
    }
    return true;
}

uint8_t BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks) {
    if (!WaitCardReady(SD_READY_TIMEOUT_MS)) return MSD_ERROR;   // 待機に戻ってから始める
    if (HAL_SD_WriteBlocks_DMA(&hsd, (uint8_t *)pData, WriteAddr, NumOfBlocks) != HAL_OK) return MSD_ERROR;
    return MSD_OK;
}
```

**512 バイトにそろえる**: 書くデータの場所(ファイルの中の位置とメモリの番地)がセクタ(512 バイト)の区切りにそろっていないと，`sd_diskio` は 1セクタずつ別のバッファにコピーして書く遅い道を通る．ログのファイルの先頭を 512 バイトちょうどにすると，追記が 1.2 秒 → 0.08 秒になった．

### 4.16 SD カード(2): 走りながら流す(`interface/sdcard.c`)

`f_write` は書き終わるまで戻らず，SD は時々数百 ms 待たせる(最大 0.6 秒ほど)．走っている最中にメインがそこで止まると，境界の処理が遅れる．そこで，**DMA を始めるだけで戻る**書き方を作った．

**考え方**: FatFs を通さずにカードのセクタへ直接書けば待たずに済む．ただし，ファイルがカードのどのセクタにあるかは FatFs しか知らない．そこで，

1. 先にファイルを 8MB まで伸ばして，場所を確保する．
2. FatFs の高速シークの「クラスタの対応表(cltbl)」を作らせる．
3. ファイルの中の位置 → カードのセクタ番号は，その表から自分で計算する．

```c
// 先に伸ばして領域を確保する(書き込みで開いたファイルは、終わりより先へ f_lseek すると伸びる)
FRESULT res = f_lseek(&s_file, reserve);
...
// クラスタの対応表を作る
s_clmt[0] = STREAM_CLMT_ITEMS;
s_file.cltbl = s_clmt;
res = f_lseek(&s_file, CREATE_LINKMAP);
```

```c
// ファイルの中の位置 off の、カードのセクタ番号と、そこから連続しているセクタ数
static bool OffsetToSector(uint32_t off, uint32_t *sector, uint32_t *contig) {
    FATFS *fs = s_file.obj.fs;
    uint32_t clsz = (uint32_t)fs->csize * 512u;
    uint32_t cl = off / clsz;              // 何番目のクラスタか
    uint32_t in_cl = (off % clsz) / 512u;  // クラスタの中の何番目のセクタか
    const DWORD *t = &s_clmt[1];           // (クラスタ数, 始まりのクラスタ) の組の並び
    while (t[0] != 0) {
        if (cl < t[0]) {
            uint32_t clst = t[1] + cl;
            *sector = fs->database + (clst - 2u) * fs->csize + in_cl;
            *contig = (t[0] - cl) * fs->csize - in_cl;
            return true;
        }
        cl -= t[0];
        t += 2;
    }
    return false;
}
```

FAT のデータ領域の最初のクラスタの番号は 2 なので，セクタ番号 = データ領域の始まり + (クラスタ番号 − 2) × クラスタの大きさ + クラスタの中の位置．領域が途中で途切れていても(断片化)，連続している所までで DMA を分ける．

**書き込みの状態**: メインは待ちのループの中で `SDCard_StreamPoll()` を何度も呼ぶ．呼ぶたびに今の状態を見て，次に進めるだけで，すぐ戻る．

```
IDLE --StreamWrite--> WAIT --カードが TRANSFER--> DMA --DMA 終わり--> WAIT --残り 0--> IDLE
                        ^                                              |
                        +------------- 残りあり(次の連続した所) ---------+
```

```c
case STREAM_DMA:
    if (HAL_SD_GetState(&hsd) != HAL_SD_STATE_READY) return 1;          // まだ送っている
    ...
    s_stream_state = STREAM_WAIT;     // カードが書き終わる(PROGRAMMING → TRANSFER)のを待つ
    return 1;
case STREAM_WAIT:
    if (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER) return 1;  // カードが書いている
    if (s_stream_left == 0) { s_stream_state = STREAM_IDLE; return 0; }
    StreamStartRun();                  // 次の連続した所の DMA を始める
```

最後に `f_truncate` で書いた所まで縮めて閉じる．こうすると，`.ioc` の FatFs の設定(`_USE_EXPAND`)を変えずに済む．

### 4.17 ロガー: ダブルバッファと YLOG2(`app/logger.c`)

ロガーは「列の名前と，値の入っている変数の場所」を登録しておき，割り込みが tick ごとに値を写す．

```c
Logger_AddField("target", &d->target_mm_s);   // 名前と、値の入っている変数の場所
Logger_AddField("vl", &d->vl);
...
```

**走りながら流すときは，バッファを2つのブロック(24KB ずつ)に分けて交互に使う**．割り込みが片方に行を足し，いっぱいになったらもう片方へ移る．メインはいっぱいになったブロックを SD へ送る．

```c
// 流す方式の1行(ISR)
static void SampleStream(void) {
    uint8_t a = s_blk_active;
    if (s_blk_full[a]) {
        uint8_t o = (uint8_t)(a ^ 1u);
        if (s_blk_full[o]) {   // 両方いっぱい(SD が追いついていない)
            s_stream_dropped++;  // その行は捨てて数える
            return;
        }
        s_blk_rows[o] = 0;
        s_blk_active = o;      // もう片方へ移る
        a = o;
    }
    float *row = BlockBase(a) + STREAM_BLOCK_HEAD + s_blk_rows[a] * s_field_count;
    for (uint32_t i = 0; i < s_field_count; i++) row[i] = *s_fields[i].value;
    if (++s_blk_rows[a] >= s_blk_max_rows) s_blk_full[a] = true;
}
```

- 34 列・5ms ごとなら，1ブロックは約 0.9 秒ぶん．SD が 1 ブロックを書くのに約 0.9 秒以上かかって初めて行が欠ける．実測は平均 16〜30ms，最大 234ms で，十分に余裕がある．
- 割り込みは送っている最中のブロックには書かない．`s_blk_full` が「割り込みが書いてよいか」の印になっている．
- 両方いっぱいのときは，待たずに捨てて数える(割り込みの中では待てない)．数は `dropped` として表示され，ログの `time_s` の飛びで PC からも分かる．

**ブロックの中身**: 頭に [magic "YBLK"，通し番号，行数，列数] を書く．PC は通し番号が続いている所まで読むので，電源が途中で切れても，そこまでのログは読める．

**ファイルの先頭は 512 バイト**: 列名を書いた後ろを空白で埋めて，ちょうど 512 バイトにする．続くブロックがセクタの区切りから始まり，そのまま DMA で書ける．

```c
// 改行を含めて LOG_BIN_HEADER_BYTES(512)になるよう、最後の改行の前を空白で埋める
while (n < total - 1u) s_header_buf[n++] = ' ';
s_header_buf[n++] = '\n';
```

### 4.18 イベントの記録(`app/logger.c`，`app/log_event.h`)

「境界で壁を読んだ」「スラロームを始めた」のような**出来事**を，ログの列として残す．

```c
void Logger_Event(uint16_t code, float a, float b, float c, float d, float e) {
    if (s_state != LOGGER_RECORDING) return;
    // メインからも ISR からも呼べるよう、入れる間だけ割り込みを止める
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t next = (s_ev_head + 1u) % EVENT_FIFO_SIZE;
    if (next == s_ev_tail) {
        s_ev_dropped++;                      // いっぱい: 捨てて数える
    } else {
        float *ev = s_ev_fifo[s_ev_head];
        ev[0] = (float)code;  ev[1] = a;  ...  ev[5] = e;
        s_ev_head = next;
    }
    __set_PRIMASK(primask);                  // 元の状態に戻す(割り込みの中から呼ばれても壊さない)
}
```

割り込みは記録する行ごとに，順番待ちから1つ取り出して `ev`〜`ev_e` の列に入れる(なければ 0)．1行に入るのは1つなので，同時に起きた出来事は次の行からずれて入る(5ms ごとの記録なら 5ms ずつ)．

- `__disable_irq()` ではなく PRIMASK を保存して戻すのは，割り込みの中(制御の ISR)から呼ばれたときに，割り込みを勝手に有効に戻さないため．
- 番号と中身の名前は `log_event.h` の1か所で決める．PC の `get_log.py` はこのファイルを読んで，`STEP x=1 y=0 ...` のような文字にする．機体と PC で表を二重に持たないので，ずれない．

### 4.19 PC でログを読む(`tools/get_log.py`)

```python
if magic == b"YLOG2":
    lines = data[:YLOG_HEADER_BYTES].split(b"\n")
    count = int(lines[1])
    names = lines[2].decode("ascii").strip().split(",")
    block_bytes = int(lines[3].strip())
    pos = YLOG_HEADER_BYTES
    seq = 0
    while pos + 16 <= len(data):
        blk_magic, blk_seq, n_rows, n_cols = struct.unpack_from("<4I", data, pos)
        if blk_magic != YBLK_MAGIC or blk_seq != seq or n_cols != count:
            break  # 書かれていない所 (または壊れたブロック)
        values = struct.unpack_from(f"<{n_rows * count}f", data, pos + 16)
        rows.extend(values[r * count:(r + 1) * count] for r in range(n_rows))
        pos += block_bytes
        seq += 1
```

ファイルは 8MB まで伸ばしてから書くので，後ろには書かれていない所が残ることがある(最後は縮めるが，途中で電源が切れた場合など)．magic と通し番号が続いている所までを読む．

**マイコンで CSV にしない理由**: 文字にする処理は遅く，ファイルも数倍の大きさになる．マイコンは float をそのまま書き，変換は PC がやる．

---

## 5. 開発の経過(時系列)

### 5.1 立ち上げ(2026-06-27 〜 10-01)

| 日付 | コミット | 内容 |
|---|---|---|
| 06-27 | 70e65f4，aa8f1ec | L チカ，UART の Hello World |
| 07-01 | 0e67f1b〜a73998d | ジャイロの Who-Am-I，値の取得，関数化 |
| 07-02 | cdc9920 | 書き込めるがモーターが回らない状態 |
| 09-21 | 0a34f6a | モーターとエンコーダが動いた |
| 09-26 | b04f14d〜0f3016a | タイマー割り込みで動かない問題を解決し，センサー値を取得．SD 以外の動作確認 |
| 09-29〜10-01 | 5c87b12，e435ae7 | センサーと機体全体の動作確認 |

### 5.2 層の整理(10-02，e6287fa)

- **目的**: CubeMX の平らな構成から，3.1 の層構成へ移す．
- **実装**: `global.h` から HAL への依存を外した．ISR に直書きしていた IR センサーの読み取りを `interface/sensor.c` へ切り出し，ISR は `Sensor_ReadAll()` を呼ぶだけにした．

### 5.3 速度制御の骨組みとモード選択(10-02，7811903)

- 汎用の PID コア(`logic/control/pid`)，運動学とオドメトリ，左右輪の速度 PID，1kHz の `App_ControlTick()` を作った．
- 右エンコーダとボタンでモードを選ぶ仕組みと，TEST モード(センサー・ジャイロ・自己位置の表示)を作った．

### 5.4 速度制御・フェイルセーフ・ロガー(10-03，3a2a287)

- **速度制御**:
  - 出力を電圧にし，FF(摩擦 + 速度 + 加速度，左右別)+ PID にした．
  - 床の上で取ったログからゲインと FF を合わせた．
  - 止まっているときは積分をリセットするようにした．
- **台形プロファイル**(`logic/control/velocity_profile`):
  - 減速の区間では，残りの距離でちょうど v_end になる減速度を毎 tick 計算し直す．
  - 減速度の上限は指定の 1.5 倍．
  - 固定の減速度だと，離散化の誤差で速度が残ったまま終点に着いてしまうため．
- **フェイルセーフ**: 低電圧，速度の偏差，角速度のどれかが続けて大きかったら止めて，そのまま止まった状態を保つ(ラッチ)．
- **ジャイロ**: ±2000dps にし，バースト読み出しを ISR へ移した．起動時にゼロ点を補正する．
- **ロガー**: divergence_v3 のログ機能を C へ移植した．`tools/get_log.py` が CSV で受け取る．
- **エンコーダの換算**: IEH2-4096 ×4 逓倍，13:42，φ23.35 に合わせた．

### 5.5 SD への保存・角速度制御・超信地旋回(10-05，7069d3e)

- **SD**:
  - SDIO 4bit + DMA と FatFs を有効にした．連番のファイルを作り，送ったファイルを管理する(`interface/sdcard.c`)．
  - SD_DUMP モードで，SD の中身を UART 経由で PC に送る．
- **カードが busy の間のエラー**: `FR_INVALID_OBJECT` が出るので，BSP_SD の `__weak` 関数を上書きして避けた．
- **制御**: 車輪速度ループの外側に，ジャイロの角速度 PI を足した．
- **試験**: PIVOT の試験モードを作った．

### 5.6 迷路の logic 層とシミュレータ(10-06，b806cec〜b5a9b7b)

- **目的**: 機体が使えない間に，迷路の判断を移植して PC で確かめる．
- **実装**:
  - BlueEyes から次の4つを移植した: 壁の地図(`wall_map`)，向き付き Dijkstra(`dijkstra`)，足立法の歩数マップ(`step_map`)，探索プランナー(`search_planner`)．
  - **`tools/maze_sim`**: 同じ C のソースを PC でコンパイルする．CLI と pygame の GUI(DLL を ctypes で呼ぶ)がある．
  - **大会の迷路を取り込む**: 公式の迷路図(画像)を classic 形式に変換する(`maze_from_image.py`，`maze_batch.py`)．2020〜2025 年の 92 枚を `my_mazes/` に入れ，印刷された最短歩数と照らし合わせて確かめた(`maze_verify.py`)．
- **結果**: 帰りに既知の区画へ入るときのコストの上乗せ `MAZE_COST_KNOWN_CELL_RETURN`(仮に 3)は，乱数の迷路では逆効果だった．全日本の 61 迷路では，最短を見つけた数を 10 → 21 に戻す効果があった．乱数の迷路と大会の迷路で結論が逆になるので，調整は大会の迷路で評価する．
- **保留**: 帰りの探索の A 案．探索用の見方での最短経路の候補の上で，未知の通路の両側の区画を目的地にする．

### 5.7 LED での知らせ方(10-06，f8ceb68，41bfdc4)

- 直結 LED に位置の名前を付けた(右前，左，左前，右，右後ろ，左後ろ)．
- フェイルセーフは，原因の番号 n を直結 LED の点滅で示す．電圧低下は右後ろで知らせる．
- SD への保存は，成功なら全部を 0.5 秒点け，失敗なら左後ろを 0.1 秒周期で点滅させる．

### 5.8 超信地旋回の追従の改善(10-06，79e996b)

- **経過**: PIVOT のログで，ジャイロの角速度が (vr − vl)/57.90 の 0.912 倍しかなかった．
- **対策 1**: `TREAD_WIDTH_MM` を 57.90 → 63.50(実効値)にした．その後のログ2本で，比は 0.995 / 1.000 になった．
- **対策 2**: 超信地旋回のときだけ足す摩擦 FF(`PIVOT_FF_FRIC`)を作った．タイヤが横にこすれるので，直進用の FF では 0.75〜0.84V 足りなかった．
- **結果**:
  - 角速度の補正の最大は 133 → 約 60 dps に下がった．
  - 車輪の速度と目標の比は 0.79〜0.88 → 0.93〜1.05 になった．
  - 右回りの振動が半分になった．

### 5.9 角度の制御を常時有効に・ジャイロのゼロ点の測り直し・手かざし(10-07，4e93154)

- 目標の向き θ_ref を持ち，止まっている間も角度の P + 角速度の P で向きを保つようにした．
- **ジャイロのゼロ点**: 起動時の補正だけでは数 dps ずれ(ログで −3〜−5.6dps)，向きの制御が機体を曲げていた．そこで，走り出す直前に 0.5 秒かけて測り直す(ISR の中で平均)ようにした．
- 試験の走り出しを手かざしにした．

### 5.10 位置の追従・モードの階層化・宴会芸モード(10-07，7eb8d14)

- **位置の制御**: 目標の距離と，進んだ距離(左右の平均)の差に P をかけて並進の目標に足す．
  - 入れる前は，止まる直前に静止摩擦に負けて −4.5mm のずれが残っていた．
- 旋回用の摩擦 FF を，回る向きでも分けて4つにした．
- モードを TEST / SD の階層にした．
- **PARTY(宴会芸)モード**: 床ごと回されても，ジャイロで同じ方向を向き続ける．

### 5.11 タイヤの直径とトレッドを走行に合わせる(10-07，9ec31a8)

- **経過**: 直進 1080mm を 300mm/s と 800mm/s で走らせると，どちらも約2%(21〜22mm)行き過ぎた．
- **判断**: 行き過ぎが速さによらないので，滑りではなく換算のずれとした．
- **対策**:
  - `WHEEL_DIAMETER_MM` を 23.35 → 23.81 にした．
  - トレッドも同じ割合で 63.50 → 64.75 にし，ジャイロとの関係を保った．
  - 加速度 FF を 10% 上げた．加速中の I 項が加速度に比例して正だったため．
- **結果**: 次のログで，止まった位置は目で見てほぼぴったりになった．

### 5.12 最短経路を走行時間で選ぶ(10-07，df86e29)

- **logic 層に `run_path` を足した**:
  - 最短走行の指令: 直進は半区画単位，ほかに小回り 90，大回り 90/180．
  - 台形加速での走行時間の見積もり．
  - 区画の経路を大回りに置き換える処理．
- **`time_dijkstra` を足した**: 走行時間が最短の経路を探す．ノードは境目 × 向き × 速度の種類で，ゴール側から逆向きに探す．
- **大回りの形**: 区画の中心から中心へ曲がる(90° は 0R0，180° は 0RR0)．
- **シミュレータ**: GUI で最短走行の経路を表示し，再生できるようにした．

### 5.13 車輪速度の P と右の摩擦 FF(10-07，f0f2303，0b99539)

- **`VELOCITY_KP` 0.0094 → 0.015**: ステップを入れると，立ち上がりの間に I 項が溜まって 20% 行き過ぎていた．P を強めて，立ち上がりを P に受け持たせた．行き過ぎは 12% に減った．
- **`VELOCITY_FF_FRIC_R` 0.53 → 0.16**: 右の車軸のねじを締め直したら駆動系が軽くなり，巡航中の I 項が −0.37V になったため．
- **結果**: 直進 300mm/s で，位置のずれ ±0.3mm，向きのずれ 0.4° 以内になった．
- **`ANGLE_KP` 15 → 25**: 超信地旋回で，FF の足りない分を受けるのに 2〜3° のずれが要っていた．25 にすると 1〜2° に減り，揺れは出なかった．

### 5.14 壁センサー・壁の判定・壁の制御(10-07，1d948fc)

- **SENSOR_LOG モード**: ボタンを押すと，3秒ぶんの壁センサーの値を記録する．
- **壁の判定**(`logic/wall_sense`):
  - 区画の境界に止めて測った値の，「壁なしの最大」と「壁ありの最小」の真ん中をしきい値にした: L 180，R 150，FL+FR 220．
  - 前の壁を FL+FR の合計で見るのは，FR が壁ありでも小さく，機体が少し斜めでも合計はあまり変わらないため．
- **壁の制御**: 直進中，左右の値の基準からのずれで目標の向きを動かす．
- **壁の切れ目・柱**: 2mm 進む間に値が 15 を超えて変わった側は，40mm の間使わない．
  - 比べる間隔は初め 5mm だったが，切れ目を見つけるまでに補正が漏れるので 2mm にした．
  - ログでは，切れ目では 5mm で 30〜170 変わり，壁に寄っていくだけなら 5mm で 2 程度だった．
- **ユーザーの指摘**: 壁がなくなったときに，ない壁に吸い寄せられるのでは？ → 切れ目の検出で，その側を使わないようにした．
- **方針**: センサーの値を mm に直すのは時間がかかるのでしない(ユーザーの判断)．値の単位のまま扱い，ゲインはログで合わせる．

### 5.15 探索走行・最短走行(10-07，bbfbcf7)

- **探索**: BlueEyes の searchB と同じ流れにした．
  - start_sequence: 右 90° → 尻当て → 左 90° → 尻当て で，区画の真ん中に合わせる．
  - 最初の半区画を走り，境界で壁を読んで経路を計算し直す．
  - 直進は止まらない．曲がるときは真ん中で超信地旋回する(このときはまだ)．
  - 180° のときは，後ろと横に壁があれば尻当てを2回する．
  - ゴールまで行けたら，地図を flash の最後のセクタに保存する．このため，リンカースクリプトの FLASH を 896K にした．
- **最短走行**: flash の地図の既知の壁だけで Dijkstra を計算し，続く直進をまとめて 800mm/s で走る．
- **壁の制御を作り直した**:
  - 初めは「誤差に比例して向きを回す速さ」の形だった．大きく寄ったときに向きが 8.7° 回ったまま残り，曲がった後に斜めのまま壁にぶつかった．
  - 「迷路の軸の向き + 横のずれに比例したオフセット」の形に変えた．横のずれが 0 に戻ればオフセットも 0 に戻るので，向きのずれが残らない．
- **尻当て**: 時間で下がる方式にした(800ms では短く，1200ms にした)．押し当てている間は向きと位置の補正を止める．
- **ログ**: 時間で区切って止まり，SD にバイナリで追記する方式にした．先頭を 512 バイトにそろえて DMA でまとめて書けるようにすると，追記が 1.2 秒 → 0.08 秒になった．
- **細かい調整**:
  - 最初の半区画は 44mm で試した後，尻当て後の前進を 40mm にした(ログで真ん中から壁まで 39〜40mm)．
  - スタートでジャイロの積算角をリセットする．
  - 加速度 3000 では，真ん中で止まる減速が強すぎて安定しなかったので 2000 にした．
- **迷路の座標**: BlueEyes と合わせた．x が東，y が北．試しの迷路は 5×7 で，ゴールはスタートの右隣(1,0)．

### 5.16 スラローム(未コミット)

- **目的**: 探索で止まらずに曲がる(小回り 90°)．最短走行の大回りも用意する．
- **実装**:
  - `logic/control/slalom`: 並進の速さ v を保ったまま，角速度を台形で動かして曲がる．
    - 前後のオフセットは，機体の ISR と同じ `VelocityProfile` を 1ms 刻みで進めて計算する．
    - 小回り 90: 境界の真ん中 → 隣の境界の真ん中(span = 半区画)．
    - 大回り 90: 区画の中心 → 斜め隣の中心(span = 1区画)．
    - 大回り 180: 横にちょうど1区画移る ω を二分法で求める．pre = 区画 − 一番奥まで進んだ距離，post = pre + 出口の前後の位置．
  - SLALOM の試験モード: s90 / l90 / l180 × 右 / 左 をクリックで選ぶ．
  - 探索で PIVOT / SMALL，最短走行で PIVOT / SMALL / LARGE を，走る前にクリックで選ぶ(maze_sim と同じ3種)．
- **経過**:
  - 最初の試験で，5本のうち2本が柱に当たった．手で置いたときのずれが原因と考え，試験の前に尻当てをするようにした．
  - 探索のスラロームの曲がり始めが遅れている．境界で経路を計算する `Step` に 20〜25ms かかる．小回りの前のオフセット(約 12mm = 500mm/s で 24ms)に近い．A/B/C の対策案は未決定．

### 5.17 走りながら SD へ流すログとイベント(未コミット)

- **目的**: 最短走行のログが RAM(48KB)に収まらない．走行全体を止まらずに記録したい．
- **実装**:
  - **`interface/sdcard`**: 待たない書き込み(`SDCard_Stream*`)．
    1. ファイルを先に 8MB 伸ばす．
    2. 高速シークの cltbl からセクタ番号を計算する．
    3. `HAL_SD_WriteBlocks_DMA` を始めるだけで戻る．
  - **`logger`**: 24KB のブロック2つを交互に使う(YLOG2 形式)．`Logger_StreamBegin` / `Poll` / `End`．
  - **イベント**: `Logger_Event(code, a〜e)` を 32 個の FIFO に入れ，ISR が次に記録する行の `ev`〜`ev_e` 列に1つずつ入れる．番号は `app/log_event.h` で決める．
  - **`run_log`**: 探索，最短走行，PARTY で同じ全部の列(27列 + イベント 6列，5ms ごと)を記録する．
  - 試験モードは今までどおり RAM に記録し，走った後に SD へ保存する．SD のファイルはすべて `.bin` にし，CSV は PC が作る．
  - **STREAM_TEST モード**: モーターを動かさずに，流す方式を確かめる．
  - 記録は手かざしで始めた時から(常に取るのではなく)．
- **結果**: PARTY で 46 秒，116 秒，134 秒走らせ，行の欠けはなかった．書き込みは平均 16〜30ms，最大 234ms で，1ブロックは約 0.9 秒ぶんなので余裕がある．
- **PARTY の拡張**: 向きに加えて位置も保つようにした．押されてから戻るまで 0.12〜0.25 秒だった．

### 5.18 前壁補正・壁切れ補正(未コミット)

- **前壁補正**(探索の小回り):
  - 曲がる区画の奥に壁があるとき，距離で決めた曲がり始めの位置の前後 15mm の間で，FL+FR が `SLALOM_FRONT_REF_SUM`(仮に 400)に届いた瞬間に曲がり始める．
  - 結果はイベント `FRONT_TRIG` に残す．
- **壁切れ補正**(進む方向の位置):
  - 直進中に横の壁が切れた瞬間は，境界を `WALL_EDGE_POS_MM`(仮に 78mm)過ぎた所にいるはずとみなす．ずれていた分だけ次の境界をずらす．
  - 予想から ±30mm を超えてずれた壁切れは使わない．1回に直す量は ±20mm まで．
  - 結果はイベント `EDGE_CORR` に残す．
- **残り**: 400 と 78 は仮の値で，迷路で走ったログで合わせる．

### 5.19 壁の制御のゲインを速さに合わせる(未コミット)

- ゲインを `WALL_KP_DEG × 500 / v` にした(100mm/s より遅いときは 100 として計算)．
- 横に戻る速さは v × オフセット なので，真ん中に戻るまでの時間が速さによらず同じになる．

### 5.20 PC ツール: turn_sim，log_viewer(未コミット，2026-10-08)

- **turn_sim**(`tools/turn_sim.py`): `turn_sim_classic2.py` を作り直したもの．
  - 機体と同じ計算(params.h を読み，同じ台形・同じオフセット)．スリップ K=0 なら出口のずれは 0.00mm で一致した．
  - **スリップアングル**: 1次遅れ dβ/dt = (K·v[m/s]·ω[rad/s] − β)/C．C = 0 なら遅れなし．
  - 結果を表示する: 出口のずれ，柱との距離，ずれを消す `*_ADJ`(params.h に貼れる形)．
  - **実機のログを重ねる**: 車輪とジャイロで作った軌道と，壁センサーで見た横の位置の差から K と C を合わせる．
  - **今のログで合わせた結果**(turn_0001, 0003〜0005):
    - K ≈ 0.008．
    - C は探す範囲の端(0.2s)で，決まっていない．
    - 右・左共通の右へのずれ b = +8.3mm．
    - 横センサーの傾きは，探索のログから約 7.8 AD/mm と見積もった(だいたいの値)．
- **log_viewer**(`tools/log_viewer.py`):
  - 列を選んで時系列のグラフにする．イベントの縦線と名前を描く．
  - イベントの一覧を出し，クリックでその時刻へ飛ぶ．
  - 迷路の上に軌道を描き，カーソルの位置を示す．
  - 再生と停止(速さ ×0.05〜×4，スライダー)．
  - 起動するとログを選ぶ画面が出る．
- **迷路の壁の表示**:
  - 読んだ壁(STEP イベント)をカーソルの時刻までの分だけ描く．
  - 機体の地図も描く．地図は，機体が走りの終わりに `LOG_EV_MAP_CELLS` で 8区画ずつログに入れる．CSV の有効数字 6桁のため，1つの値には 16bit まで詰める．
- **再生の描画**: グラフ全体を描き直すと1コマ約 0.3 秒かかったので，カーソルだけを重ねて描く方式(blit)にした．約 74ms になった．

### 5.21 速さを選ぶ・番号を 1 から(未コミット，2026-10-08)

- モードの番号を 1 から(n 番 = LED n, n+1)にし，0 番の全点灯は無くした．
- **モードを決めた直後に速さを選ぶ**: 操作はモード選択と同じ．最初は params.h の値に一番近いもの．

  | モード | 選べる値 |
  |---|---|
  | 探索，STRAIGHT，SLALOM(s90) | 300/400/500/600 mm/s |
  | 最短走行(直進の最高速度) | 600/800/1000/1200 mm/s |
  | PIVOT | 180/360/540 dps |

- **小回りの速さを変えたとき**: ω ∝ v，α ∝ v² にして，曲がる形を保つ(`Slalom_ScaleToSpeed`)．オフセットと前壁補正の位置はほぼ同じで，横加速度は v² で増える(300: 0.14G，500: 0.40G，600: 0.58G)．

---

## 6. 調整のやり方(手順集)

### 6.1 車輪速度の FF(`VELOCITY_FF_*`)

1. **摩擦と速度の項**: 定常で走らせて，2つの速さで電圧を測る．直線 `電圧 = FRIC + GAIN × 速度` で近似する．左右で分ける．
2. **微調整**: 巡航中の I 項を見る．I 項が正なら FF が足りない(FRIC を上げる)．0 付近になれば合っている．
3. **加速度の項**:
   - 台形の加速中・減速中の電圧から，摩擦と速度の分を引き，加速度で割る．加速と減速で平均すると，摩擦の影響が打ち消される．
   - その後，加速度を変えたログで，加速中の I 項が加速度に比例しているかを見る．比例して正なら足りない．
4. **駆動系が変わったら合わせ直す**: 車軸のねじを締め直しただけで，右の FRIC は 0.53 → 0.16 に変わった．分解したり，ぶつけたりした後は I 項を見る．

### 6.2 超信地旋回の摩擦 FF(`PIVOT_FF_FRIC_*`)

1. PIVOT の試験でログを取る．回る向きとは逆の I 項が出ていれば多すぎ，同じ向きなら足りない．
2. 2つの値で取ったログの I 項から，直線で補間して I 項が 0 になる値にする．
3. 左回り / 右回りで分ける．右回りはいつも左回りの後に回しているので，順番の影響が混ざっている可能性がある．

### 6.3 寸法(タイヤの直径・トレッド)

- **直径**: 決めた距離を走らせ，行き過ぎた量を測る．速さを変えても同じ割合なら換算のずれとみなして，直径を比で直す．
- **トレッド**: PIVOT のログで，ジャイロの角速度 / ((vr − vl)/TREAD) の比を見て合わせる．直径を変えたら，トレッドも同じ割合で変える．

### 6.4 ゲイン

- **車輪速度の P**: VEL_PID のステップで，立ち上がりに I 項が溜まって行き過ぎるなら P を上げる．
- **角度の P**: 旋回中の向きのずれを見る．止まった後に揺れが出ない範囲で上げる．
- **振動が出たら**: 閾値・不感帯ではなくゲインで直す(3.2)．

### 6.5 壁センサー

- **しきい値**: SENSOR_LOG モードで，境界に止めて「壁あり / 壁なし」を測る．真ん中にする．
- **WALL_REF**: 真ん中にいるときの L / R．走行のログのスタート直後の値にした．
- **壁の制御のゲイン**: STRAIGHT のログ(壁センサーの列あり)で，横のずれの戻り方を見る．
- **壁切れの位置 `WALL_EDGE_POS_MM`**: 迷路を走ったログの `EDGE_CORR` イベント(expected，edge_pos)を見て合わせる．
- **前壁の `SLALOM_FRONT_REF_SUM`**: `FRONT_TRIG` イベント(diff，sum)を見て合わせる．

### 6.6 スラロームのオフセットとスリップ

1. SLALOM の試験で，右・左の両方のログを取る．曲がった後の通路は，左右に壁がある所にする．
2. turn_sim でログを開き，「傾きを探索のログから見積もる」で横センサーの傾きを入れる．
3. 「K と C をログに合わせる」を押す(共通の横のずれ b も合わせる)．
4. 出てくる `*_PRE_ADJ_MM` / `*_POST_ADJ_MM` を params.h に貼る．
5. もう一度走らせて確かめる．

### 6.7 迷路の判断のコスト

- `tools/maze_sim` で，全日本の迷路(`my_mazes/`)を使って評価する．乱数の迷路と結論が逆になることがある(5.6)．

---

## 7. ログの仕組みと使い方

### 7.1 形式

| 形式 | 使うところ | 中身 |
|---|---|---|
| YLOG1 | 試験モード(RAM に記録してから保存) | `"YLOG1\n<列数>\n<列名>..."` + float32 × 列数 × 行数 |
| YLOG2 | 探索，最短走行，PARTY，STREAM_TEST(走りながら流す) | 512 バイトの先頭 + 24KB のブロック(`YBLK`，通し番号，行数，列数 + float32) |

- SD 上のファイル名は `<モードのフォルダ>/<名前>_NNNN.bin`．番号は `.csv` / `.bin` をまたいで重ならない．
- 間引き: 走りながら流すものは 5ms ごと．

### 7.2 イベント(`Core/Inc/app/log_event.h`)

- 1行に1つ．`ev` 列が番号で，`ev_a`〜`ev_e` が中身．
- 「`LOG_EV_名前 = 番号, // a:名前 b:名前 ...`」の行を `get_log.py` が読み，CSV に `ev_text` 列(例: `STEP x=1 y=0 ...`)を足す．
- 番号は増やすだけで，使っている番号は変えない(昔のログを読めるように)．

| 番号 | 内容 |
|---|---|
| 1〜6 | 操作と状態: MODE，RUN_TYPE，HAND_START，CLICK，GYRO_RECAL，CTRL_ENABLE |
| 10〜14 | 制御の切り替え: WALL_CTRL，WALL_USE，WALL_EDGE，EDGE_CORR，FRONT_TRIG |
| 20〜24 | 動き: STRAIGHT，PIVOT，SLALOM，MOTION_DONE，SET_VELOCITY |
| 30〜34 | 走りの単位: TURN_KIND，SETPOS，STOP_CENTER，RUN_CMD，ROUTE_CMD |
| 40〜45 | 迷路: STEP，STEP_INFO，PHASE，MAP_SAVED，ROUTE，MAP_CELLS |
| 50〜51 | 異常: FAILSAFE，TIMEOUT |

### 7.3 PC での使い方

```
python tools/get_log.py COM5                     # SD_DUMP / ロガーの送信を受け取る(.bin の隣に .csv も作る)
python tools/get_log.py --bin2csv logs/search/search_0006.bin
python tools/log_viewer.py                       # 起動してログを選ぶ
python tools/turn_sim.py                         # スラロームのシミュレーションとログの照合
```

---

## 8. PC ツール

| ツール | 役割 |
|---|---|
| `tools/get_log.py` | UART で受け取る(SD_DUMP，ロガー)．`.bin` → `.csv`(`ev_text` 付き)．同じ名前の古い CSV は上書きせず `_dupN` を付ける |
| `tools/log_viewer.py` | ログを見る GUI(時系列，イベントの一覧，迷路の上の軌道，壁，再生) |
| `tools/turn_sim.py` | スラロームのシミュレータ(機体と同じ計算，スリップ，ログとの照合，ADJ の提案) |
| `tools/yuho_common.py` | 上の2つで共通: params.h の読み込み，台形とスラロームの Python 版，ログの読み込み，軌道の作成 |
| `tools/maze_sim/` | 迷路の探索と最短経路のシミュレータ(C の logic 層を共有)，迷路図の取り込み |
| `tools/turn_sim_classic2.py` | 元の pygame のスラロームのシミュレータ(残してある) |

---

## 9. つまずいたことと教訓

| 問題 | 原因 | 対策・教訓 |
|---|---|---|
| タイマー割り込みで動かない(09-26) | (コミットの記録のみ．詳細は未記録) | — |
| SD の `FR_INVALID_OBJECT` | カードが書き込み中(PROGRAMMING)のまま次の操作をした | BSP_SD の `__weak` 関数を上書きして待つ |
| `SD: mount failed (FRESULT=1)`(FR_DISK_ERR) | 原因は未確認 | 書き込み直したら直った |
| SD への追記が 1.2 秒 | 書き込む位置がセクタの区切りにそろっておらず，`sd_diskio` が1セクタずつの遅い道を通った | 先頭を 512 バイトにそろえる → 0.08 秒 |
| `.bin` と `.csv` の番号が重なり，古い CSV が上書きされた | 番号を拡張子ごとに数えていた | 両方を見て番号を決める．PC 側も `_dupN` |
| ロガーの列の上限で vbat が落ちた | 上限 18(時刻を含む) | 列を減らした後，上限を 40 にした |
| 壁の制御で向きが 8.7° 回ったまま残った | 「向きを回す速さ」の形(積分になる) | 「軸 + オフセット」の形にした(5.15) |
| 尻当てが短い | 下がる時間が 800ms | 1200ms にした |
| 減速で不安定 | 加速度 3000 | 2000 にした |
| スラロームの試験で柱に当たる | 手で置いたときのずれ | 試験の前に尻当て |
| スラロームの曲がり始めが遅い | 境界の経路計算に 20〜25ms | 未解決(前壁補正で一部は吸収) |
| E31 のデバッガのエラー | (ユーザーが貼ったもの．書き込み環境の問題) | — |
| PC でファイルを書き換えると C の `\r\n` が本物の改行になる | Git Bash の heredoc + Python のエスケープ | エスケープを含む編集は Edit ツールで行い，後で文字列の壊れを確かめる |
| 電池が 7.06V まで下がっていた | 充電忘れ | フェイルセーフは 7.0V．走る前に充電する |

---

## 10. 今のパラメータ(2026-10-08)

詳しい理由は `Core/Inc/params.h` のコメントにある．

| 分類 | 値 |
|---|---|
| 寸法 | WHEEL_DIAMETER 23.81mm，TREAD 64.75mm |
| 車輪速度 | KP 0.015，KI 0.0376 [V/(mm/s)] |
| 速度 FF | FRIC L 0.32 / R 0.16 V，GAIN L 0.00075 / R 0.0005，ACC 0.000275 |
| 旋回 FF | CCW L 0.57 / R 0.69，CW L 0.64 / R 0.76 V |
| 角度 | ANGULAR_KP 0.5，ANGLE_KP 25，補正の上限 6 rad/s |
| 位置 | POSITION_KP 10，補正の上限 200mm/s |
| スタート | FL > 350 が 50ms |
| 壁の判定 | L 180，R 150，FL+FR 220 |
| 壁の制御 | REF L 312 / R 282，KP 0.03 deg(500mm/s のとき，∝1/v)，最大 5°，90dps |
| 壁切れ補正 | 境界 + 78mm(仮)，±30mm，最大 20mm |
| 探索 | 500mm/s(選べる)，2000mm/s²，超信地 180dps / 1800dps² |
| 尻当て | 100mm/s で 1200ms 下がる，前へ 40mm |
| 最短走行 | 800mm/s(選べる)，2000mm/s² |
| 小回り | 500mm/s・450dps・8000dps²(速さに合わせて ω ∝ v，α ∝ v²)，オフセット約 12mm |
| 大回り 90 | 600mm/s・230dps・3000dps² |
| 大回り 180 | 500mm/s・3000dps²(ω は計算で約 323dps) |
| 前壁補正 | FL+FR 400(仮)，±15mm |
| フェイルセーフ | 7.0V が 500ms，速度の偏差 500mm/s が 100ms，角速度 1500dps が 10ms |

---

## 11. 残っている課題

- **ビルドと実機での確認**: 5.16〜5.21 の変更は 752f795〜1e54424 にコミット・push 済みだが，ビルドも実機での走行もまだ．
- **迷路で確かめる**:
  - 前壁補正の `SLALOM_FRONT_REF_SUM`(400)と，壁切れの `WALL_EDGE_POS_MM`(78)を，`FRONT_TRIG` / `EDGE_CORR` のイベントで合わせる．
  - 機体の地図をログに入れる `LOG_EV_MAP_CELLS` を，実機で確かめる．
  - 大回り(l90 / l180)の試験のログを取る．
- **スラロームの曲がり始めの遅れ**: 境界の経路計算(20〜25ms)の対策を決める．
- **スリップ**:
  - K と C は，今のログでは C が決まらなかった．
  - 右・左の両方，両側に壁のある出口で取り直す．
  - 速さ(300〜600mm/s)を変えたログもあると，K と C を分けやすい．
- **最短走行の小回りでの前壁補正**: 未実装．
- **帰りの探索の A 案**: 保留．
- **センサーの値の mm への換算**: しない方針．横センサーの傾きは，探索のログからの見積もり(約 7.8 AD/mm)を使っている．
- **LED8 以降**: U6 のはんだ不良で，確実に光るのは LED1〜7．モードの番号 7 以降は LED8 を使う．
- **turn_sim_classic2.py**: 残すか消すかは未定．

---

## 12. 開発日記(2026-10-08 以降)

ここから先は，作業をするたびに時系列どおりに書き足していく．
1〜11 章(まとめ)は，内容が変わったときだけ直す(パラメータの表・残課題など)．

書き方: 日付の見出しの下に，作業ごとに「やったこと・なぜ・結果(確かめたこと / 未確認)・次にやること」．

### 2026-10-08

- **開発記録を作った**(この文書)．立ち上げから今日までの経過・設計・調整のやり方をまとめた．
  以後は作業ごとにこの日記へ書き足す．

- **push した**: feature/maze のコミット済みの 9 個(79e996b〜bbfbcf7)を origin へ push した．
  bbfbcf7 より後の変更(5.16〜5.21 と開発記録)はまだコミットしていないので，push には入っていない．

- **未コミットの変更を feature/maze にコミットした**(内容ごとに6つ: ログ，制御，スラローム，走行と UI，PC ツール，開発記録)．
  どのコミットもビルドと実機での確認をしていないので，コミットの本文に「未確認」と書いた．
  ファイル単位で分けたので，途中のコミットだけを取り出すとビルドできない可能性がある(全部そろった状態で使う)．
  `.settings/stm32cubeide.project.prefs` と `Debug/yuho.bin` は，今のソースと合っているか分からないので入れなかった．

- **push に失敗した**: 6つのコミット(752f795〜1e54424)を push しようとしたが，GitHub が
  `remote rejected (Internal Server Error)` で拒否した．最初のコミット1つだけでも同じだった．
  送るファイルは最大 48KB と小さく，GitHub の状態表示は正常だった．原因は未確認(GitHub 側の一時的な不具合か，
  リポジトリの設定かは分からない)．コミットは手元の feature/maze に残っている．時間をおいて push し直す．
  新しいブランチ feature/slalom(feature/maze と同じ位置)を作って push しても，同じ Internal Server Error だった
  (Request ID C16F:1ED705:1E1A95:2A33EC:6AC679D1)．ブランチ名の問題ではなく，この時点の GitHub への push 全体が失敗している．
  ユーザーが GitHub の画面でプルリクを許可しようとしたら error 500 だった．GitHub の API で見ると，
  プルリクは1つもなく，ブランチの保護もルールセットもなかった(リモートの feature/maze は bbfbcf7 のまま)．
  画面の操作でも 500 が出るので，GitHub 側でこのリポジトリへの書き込みが失敗している状態と考えている(未確認)．
  時間をおいて push し直したら成功した(bbfbcf7..1e54424 → origin/feature/maze)．GitHub 側の一時的な不具合だったと考えている．
  試しに作った feature/slalom は手元にだけある(リモートには作られていない)．
- **開発日記を push した**: 日記の追記をコミットして(f193907)，origin/feature/maze へ push した．
  日記は書き足したらコミットして push する．

- **開発記録に「4. 実装の解説(コードで読む)」の章を足した**: ユーザーの依頼(PID の式や SD カードの使い方のコードを入れ，
  読んだ人が分かるように)．1ms の割り込みの流れ，センサーの読み方，エンコーダ，運動学，PID のコア(アンチワインドアップ)，
  車輪速度のループと FF(電圧で考える理由，2自由度制御)，外側のループ(角度の誤差の時定数 (1+K_ω)/K_θ = 0.06 秒の導出)，
  台形プロファイル，メインと割り込みの受け渡し，壁の制御，壁切れ補正，スラロームの形，探索の流れ，フェイルセーフ，
  SD カード(FatFs の使い方，BSP の上書き，走りながら流す仕組み)，ロガー(ダブルバッファ，YLOG2)，イベントの FIFO，
  PC でのログの読み方を，実際のコードを引用して説明した．元の 4〜11 章は 5〜12 章に番号を付け直した．
