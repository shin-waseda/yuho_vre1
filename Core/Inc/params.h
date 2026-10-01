#ifndef INC_PARAMS_H_
#define INC_PARAMS_H_

// ============================================================
// 機体寸法・エンコーダ換算
// 値は仮(プレースホルダ)。実機で実測して補正すること。
// ============================================================

#define WHEEL_DIAMETER_MM      24.0f   // TODO: 実測
#define ENCODER_PULSES_PER_REV 1024.0f // TODO: エンコーダ分解能×ギア比を確認

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
// 速度PID
// ゲインは未調整のプレースホルダ。これから実機で調整する。
// 出力はPWM相当の補正量。Motor_Drive()が符号で前後進を切り替える
// ため、クランプ範囲も対称(マイナス側=後退/ブレーキ方向)にする。
// ============================================================

#define VELOCITY_KP 1.0f
#define VELOCITY_KI 0.0f
#define VELOCITY_KD 0.0f

#define VELOCITY_PID_OUTPUT_MIN -4199.0f // PWM_MAX(interface/motor.h)と同じ絶対値を想定
#define VELOCITY_PID_OUTPUT_MAX  4199.0f

// ============================================================
// モード選択
// 右エンコーダがこのパルス数だけ回転するたびにモードを1つ送る。
// ============================================================

#define MODE_SELECT_PULSES_PER_STEP (ENCODER_PULSES_PER_REV / 6.0f)

#endif
