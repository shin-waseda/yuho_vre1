#ifndef INC_PARAMS_H_
#define INC_PARAMS_H_

// ============================================================
// 機体寸法・エンコーダ換算
// エンコーダはモーター軸に付いているので、タイヤ1回転あたりの
// カウント数 = モーター1回転のパルス数 × 逓倍 × 減速比。
// ============================================================

#define WHEEL_DIAMETER_MM      23.35f  // 実測値

#define ENCODER_PPR        4096.0f // FAULHABER IEH2-4096: モーター1回転あたり
#define ENCODER_MULTIPLIER 4.0f    // TIM4/TIM8 = TIM_ENCODERMODE_TI12 (A/B両エッジで4逓倍)
#define GEAR_PINION_T      13.0f   // M0.5 (divergence_v3と同じ)
#define GEAR_SPUR_T        42.0f   // M0.5

// タイヤ1回転あたりのカウント数 (≈52935)
#define ENCODER_PULSES_PER_REV (ENCODER_PPR * ENCODER_MULTIPLIER * GEAR_SPUR_T / GEAR_PINION_T)

#define MM_PER_PULSE ((3.14159265f * WHEEL_DIAMETER_MM) / ENCODER_PULSES_PER_REV)

// エンコーダの取り付け向きにより、パルス増加方向と前進方向が
// 逆転する場合に-1.0fへ反転させる。
#define ENCODER_L_SIGN 1.0f
#define ENCODER_R_SIGN -1.0f

#define TREAD_WIDTH_MM 57.90f // TODO: 実測(左右タイヤの接地点間距離)

// ============================================================
// 制御周期
// TIM6: Prescaler=84-1, Period=1000-1 → 84MHz/84/1000 = 1kHz
// ============================================================

#define CONTROL_DT_S 0.001f

// ============================================================
// 速度制御 (2自由度制御: FF + PID)
// 出力はモーター電圧[V]。duty = 電圧 / バッテリー電圧 でPWMへ変換するので、
// 電池電圧が下がってもゲインの効き方が変わらない。
//   電圧 = FF_FRIC × sign(目標速度)   (クーロン摩擦。速度によらず一定)
//        + FF_GAIN × 目標速度         (速度に比例する分)
//        + FF_ACC  × 目標加速度       (加速に要る分)
//        + PID(目標 - 実速度)         (外乱・モデル誤差の補正)
// FFは目標だけから計算し、実測を通さない(2自由度制御)。
// Kp/Kiは、PWM単位で調整した値(Kp=5, Ki=20 [PWM/(mm/s)])を
// Vbat≈7.9V (1V≈531.5PWM) で換算したもの。
// ============================================================

#define VELOCITY_KP 0.0094f // [V/(mm/s)]
#define VELOCITY_KI 0.0376f // [V/(mm/s)/s]
#define VELOCITY_KD 0.0f

#define VELOCITY_PID_OUTPUT_MIN -BATTERY_FULL_V // [V] 対称(マイナス側=後退/ブレーキ方向)
#define VELOCITY_PID_OUTPUT_MAX  BATTERY_FULL_V

// 速度FF。この機体は左右の駆動系の重さが違うので左右で分ける。すべて床上の値。
// 定常走行時の電圧を2つの速度で測り、直線 電圧 = FRIC + GAIN × 速度 で近似した:
//   左: 0.47V @200mm/s, 0.55V @300mm/s / 右: 0.51V @200mm/s, 0.56V @300mm/s
// 微調整は定常時のI項を見る(I項が0付近になれば合っている)。
#define VELOCITY_FF_FRIC_L 0.32f     // [V]
#define VELOCITY_FF_FRIC_R 0.41f     // [V]
#define VELOCITY_FF_GAIN_L 0.00075f  // [V/(mm/s)]
#define VELOCITY_FF_GAIN_R 0.00050f  // [V/(mm/s)]

// 加速度FF係数[V/(mm/s^2)]。
// 台形プロファイル(±2000mm/s^2)の加速中・減速中の電圧から、摩擦と速度の分を
// 引いて加速度で割り、加速・減速の平均を取った値(摩擦の影響が打ち消される)。
#define VELOCITY_FF_ACC_L 0.00025f
#define VELOCITY_FF_ACC_R 0.00025f

// 迷路の1区画の長さ[mm]
#define SECTION_MM 180.0f

// 目標0かつ左右の実速度がこれ未満なら、PIDの積分を捨てて出力0にする。
// (摩擦で止まった後もI項が残り、PWMが出続けるのを防ぐ)
#define VELOCITY_STOP_RESET_MM_S 20.0f

// 電圧→dutyの変換に使うバッテリー電圧の下限[V]。電池未接続時の0割り防止。
#define VELOCITY_VBAT_MIN_V 5.0f

// ============================================================
// バッテリー電圧 (2S LiPo)
// VBAT -- R14(33k) -- VOL_CHECK(PC0) -- R15(20k) -- GND (yuho.netより)
// ADCの基準電圧(VREF+)は3.3V想定(未確認)。
// ============================================================

#define ADC_VREF_V            3.3f
#define ADC_FULL_SCALE        4095.0f
#define BATTERY_DIVIDER_RATIO ((33.0f + 20.0f) / 20.0f)

#define BATTERY_FULL_V   8.4f  // 2S満充電。起動時LEDバー表示の上端
#define BATTERY_IIR_ALPHA 0.1f // 1kHz更新でのIIR係数(時定数 約10ms)

// ============================================================
// ジャイロ (ICM-42688-P)
// ICM_Init()で±2000dpsに設定する。
// ============================================================

#define GYRO_SENSITIVITY_LSB_PER_DPS 16.4f
#define GYRO_Z_SIGN 1.0f // 反時計回り(左旋回)を正にしたい場合に実機で合わせる

// ============================================================
// フェイルセーフ
// 発動すると制御を止めてラッチする(リセットまで復帰しない)。
// しきい値は仮。実機で誤発動しない範囲に調整すること。
// ============================================================

#define FAILSAFE_LOW_VOLTAGE_V   7.0f // これを下回り続けたら停止 (セル3.5V)
#define FAILSAFE_LOW_VOLTAGE_MS  500  // 加速時の電圧降下で誤発動しないための継続時間

#define FAILSAFE_VEL_ERR_MM_S    500.0f // |目標-実速度| (左右どちらか)
#define FAILSAFE_VEL_ERR_MS      100

#define FAILSAFE_GYRO_DPS        1500.0f // |角速度Z|
#define FAILSAFE_GYRO_MS         10

// ============================================================
// モード選択
// 右エンコーダがこのパルス数だけ回転するたびにモードを1つ送る。
// ============================================================

#define MODE_SELECT_PULSES_PER_STEP (ENCODER_PULSES_PER_REV * 1.5f) // タイヤ1.5周で1モード

#endif
