#include "app/control_loop.h"

#include <math.h>
#include <stddef.h>
#include "main.h" // HAL_GetTick / HAL_Delay(App_RecalibrateGyroZ の待ち)
#include "interface/encoder.h"
#include "interface/motor.h"
#include "interface/gyro.h"
#include "interface/battery.h"
#include "logic/state_estimation/kinematics.h"
#include "logic/state_estimation/imu.h"
#include "logic/state_estimation/gyro_bias.h"
#include "logic/control/drive_model.h"
#include "logic/control/velocity_profile.h"
#include "app/failsafe.h"
#include "app/logger.h"
#include "app/log_event.h"

#define DEG_TO_RAD (3.14159265f / 180.0f)
#define RAD_TO_DEG (180.0f / 3.14159265f)
#define MM_TO_M 0.001f

static Odometry_t s_odo;
// 並進の FB の積分[m](∫e dt)と，旋回の FF の状態(F)．ISRからのみ触る．
static float s_vs_integral = 0.0f;
static DriveModelFF s_model;
static WheelVelocity s_actual;

// 目標の向き[deg](旋回の FB で使う)。ISRからのみ触る。
// App_ResetGyroAngle() でジャイロの積分角と一緒に 0 にした時点(走りの始め)を基準に、
// プロファイルの角速度を積分して進める。制御を止めても今の向きに合わせ直さない(向きのずれを捨てずに、旋回の FB で直す)。
// 壁センサーで向きを動かす壁の制御はやめた(2026-11-04)。
static float s_angle_ref_deg = 0.0f;

// 位置のループ(並進の FB の外側)。ISRからのみ触る。
// 目標の距離と進んだ距離[mm]。どちらも制御が無効の間は0にしておき、有効にした時点を基準にする。
static float s_pos_ref_mm = 0.0f;
static float s_dist_mm = 0.0f;
static volatile bool s_position_hold = true; // メインが書き、ISRが読む

// 横のずれの FB。目標の位置と推定の位置[mm](向き 0 を +x、左を +y)。目標の距離と同じく、
// 制御が無効の間は0にしておき、有効にした時点を基準にする。ISRからのみ触る。
static float s_ref_x = 0.0f, s_ref_y = 0.0f;
static float s_est_x = 0.0f, s_est_y = 0.0f;
static float s_lat_ofs_deg = 0.0f; // 目標の向きに足すずらし Δθ[deg]
// 目標の位置に対する推定の位置のずれ[mm](目標の向きの前後: + なら目標が前、横: + なら機体が左)。ISRが書き、メインが読む
static volatile float s_along_err_mm = 0.0f;
static volatile float s_lat_err_mm = 0.0f;

// 目標(並進・回転)。プロファイル実行中はISRが毎tick書き換える。
static float s_target_mm_s = 0.0f;
static float s_target_acc = 0.0f;
static float s_target_omega_dps = 0.0f;
static float s_target_alpha_dps2 = 0.0f;
static MotorPWM s_pwm = { 0, 0 };

// 走行プロファイル(直進・超信地旋回・スラロームのどれか1つ)。ISRだけが進める。
// 超信地旋回とスラロームは、角度[deg]を「距離」、角速度[dps]を「速度」として同じ台形を使う。
// スラロームは並進の目標速度をそのまま保ち、角速度だけを台形で動かす。
typedef enum {
    MOTION_STRAIGHT,
    MOTION_PIVOT,
    MOTION_SLALOM,
    MOTION_FRONT_ALIGN, // 前の壁で合わせる(プロファイルは使わない。UpdateFrontAlign が目標を決める)
} MotionType;

static VelocityProfile s_profile;
static MotionType s_profile_type = MOTION_STRAIGHT;
static float s_profile_sign = 1.0f; // 直進: +1 前進 / -1 後退、超信地旋回: +1 反時計回り
static volatile bool s_profile_active = false;
static volatile bool s_motion_done = true;

// 超信地旋回の合わせ込み(台形が終わった後、ジャイロの角度が目標に合うまで)。
// 間は s_motion_done を立てず、旋回の FB に角度の差の積分を足す。ISRが立てて下ろす(新しい指令でメインも下ろす)。
static volatile bool s_pivot_settling = false;
static float s_settle_integral = 0.0f; // ∫(θ_ref − θ)dt [rad·s]。ISRからのみ触る
static uint32_t s_settle_ticks = 0;
static float s_settle_prev_err = 0.0f; // 前のtickの角度の差[rad](0 を越えたかを見る)

// 前の壁で合わせる間。ISRが立てて下ろす(新しい指令でメインも下ろす)。
static volatile bool s_front_aligning = false;
static uint32_t s_align_ticks = 0;    // ISRからのみ触る
static uint32_t s_align_ok_ticks = 0; // 続けて合っている tick 数

// メイン→ISRへの走行指令の受け渡し。パラメータを書いてから最後にフラグを立て、
// ISRは次のtickでフラグを見て取り込む(取り込み中にメインが書き換えることはない)。
typedef struct {
    MotionType type;
    float distance; // [mm] or [deg](正の値)
    float v_max;    // [mm/s] or [dps]
    float v_end;    // [mm/s](超信地旋回は0)
    float accel;    // [mm/s^2] or [dps^2]
    float decel;    // 減速度(直進だけ別にできる。旋回は accel と同じ)
    float sign;    // 直進: +1 前進 / -1 後退、超信地旋回: +1 反時計回り
} MotionCommand;
static MotionCommand s_pending;
static volatile bool s_start_pending = false;

static GyroData s_gyro_raw = { 0, 0, 0 };
static float s_gyro_z_dps = 0.0f;
static float s_gyro_angle_deg = 0.0f; // ジャイロの積分角(起動からの累積、反時計回り正)
// メインが立て、ISRが積分角と迷路の軸の向きを0にしてから下ろす(ISRの足し込みとぶつからないよう、0にするのはISR)
static volatile bool s_gyro_angle_reset_req = false;
static GyroOffset s_gyro_offset = { 0.0f, 0.0f, 0.0f };

// ジャイロ Z のゼロ点の測り直し。メインが合計と回数を0にしてから残りtick数を書き、
// ISRが毎tick足し込んで、0になったらゼロ点を更新する。
static volatile uint32_t s_gyro_cal_ticks_left = 0;
static float s_gyro_cal_sum = 0.0f;
static uint32_t s_gyro_cal_count = 0;
static ControlDebug s_dbg;
// 走っている途中で止まっている間に、ジャイロ Z のゼロ点を測り直す(ISRからのみ触る)
static GyroBiasEst s_bias_est;

// メインループが書き、ISRが読む。フェイルセーフ発動時はISRもfalseを書く。
static volatile bool s_enabled = false;
static bool s_prev_enabled = false;     // ISRからのみ触る

void App_ControlLoop_Init(void) {
    Odometry_Reset(&s_odo);
    s_vs_integral = 0.0f;
    DriveModel_Reset(&s_model);
    FailSafe_Init();
    s_gyro_offset = ICM_GetOffset(); // ICM_CalibrateBlocking()後に呼ばれる前提
    GyroBias_Init(&s_bias_est);
    s_actual.left_mm_s = 0.0f;
    s_actual.right_mm_s = 0.0f;
}

void App_SetTargetVelocity(float mm_s) {
    // プロファイルを先に止めてから目標を書く(ISRが上書きしないように)。
    s_start_pending = false;
    s_profile_active = false;
    s_pivot_settling = false;
    s_front_aligning = false;
    s_motion_done = true;
    s_target_acc = 0.0f;
    s_target_omega_dps = 0.0f;
    s_target_alpha_dps2 = 0.0f;
    s_target_mm_s = mm_s;
    Logger_Event(LOG_EV_SET_VELOCITY, mm_s, 0.0f, 0.0f, 0.0f, 0.0f);
}

void App_StartStraight(float distance_mm, float v_max, float v_end, float accel) {
    App_StartStraightAD(distance_mm, v_max, v_end, accel, accel);
}

void App_StartStraightAD(float distance_mm, float v_max, float v_end, float accel, float decel) {
    s_pending.type = MOTION_STRAIGHT;
    s_pending.distance = fabsf(distance_mm);
    s_pending.v_max = v_max;
    s_pending.v_end = v_end;
    s_pending.accel = accel;
    s_pending.decel = decel;
    s_pending.sign = (distance_mm >= 0.0f) ? 1.0f : -1.0f;
    s_motion_done = false;
    s_start_pending = true; // 最後に立てる
    Logger_Event(LOG_EV_STRAIGHT, distance_mm, v_max, v_end, accel, decel);
}

void App_StartPivot(float angle_deg, float omega_max_dps, float alpha_dps2) {
    s_pending.type = MOTION_PIVOT;
    s_pending.distance = fabsf(angle_deg);
    s_pending.v_max = omega_max_dps;
    s_pending.v_end = 0.0f;
    s_pending.accel = alpha_dps2;
    s_pending.decel = alpha_dps2;
    s_pending.sign = (angle_deg >= 0.0f) ? 1.0f : -1.0f;
    s_motion_done = false;
    s_start_pending = true; // 最後に立てる
    Logger_Event(LOG_EV_PIVOT, angle_deg, omega_max_dps, alpha_dps2, 0.0f, 0.0f);
}

void App_StartFrontAlign(void) {
    s_pending.type = MOTION_FRONT_ALIGN;
    s_pending.distance = 0.0f;
    s_pending.v_max = 0.0f;
    s_pending.v_end = 0.0f;
    s_pending.accel = 0.0f;
    s_pending.decel = 0.0f;
    s_pending.sign = 1.0f;
    s_motion_done = false;
    s_start_pending = true; // 最後に立てる
}

float App_GetPositionError(void) {
    return s_along_err_mm;
}

float App_GetLateralError(void) {
    return s_lat_err_mm;
}

void App_StartSlalom(float angle_deg, float omega_max_dps, float alpha_dps2) {
    s_pending.type = MOTION_SLALOM;
    s_pending.distance = fabsf(angle_deg);
    s_pending.v_max = omega_max_dps;
    s_pending.v_end = 0.0f;
    s_pending.accel = alpha_dps2;
    s_pending.decel = alpha_dps2;
    s_pending.sign = (angle_deg >= 0.0f) ? 1.0f : -1.0f;
    s_motion_done = false;
    s_start_pending = true; // 最後に立てる
    Logger_Event(LOG_EV_SLALOM, angle_deg, omega_max_dps, alpha_dps2, s_target_mm_s, 0.0f);
}

bool App_IsMotionDone(void) {
    return s_motion_done && !s_start_pending;
}

float App_GetTargetDistance(void) {
    return s_pos_ref_mm;
}

// ---- 壁切れ ----
// 補正には使わず、イベント(LOG_EV_WALL_EDGE)に残すだけ。後でログから推定のずれを測るのに使う。
typedef struct {
    bool above;          // しきい値より上(壁あり)
    float above_from;    // 壁ありになった目標の距離
} EdgeTrack;

static EdgeTrack s_edge_l = { false, 0.0f };
static EdgeTrack s_edge_r = { false, 0.0f };

// ISR から呼ぶ。直進していないときは追わない(見つけ直しになる)。
static void UpdateWallEdge(EdgeTrack *t, uint16_t value, uint16_t th, uint8_t side, bool moving) {
    if (!moving) {
        t->above = false;
        return;
    }
    if (value > th) {
        if (!t->above) {
            t->above = true;
            t->above_from = s_pos_ref_mm;
        }
        return;
    }
    if (t->above) {
        t->above = false;
        if (s_pos_ref_mm - t->above_from >= WALL_EDGE_MIN_WALL_MM) {
            Logger_Event(LOG_EV_WALL_EDGE, (float)side, s_pos_ref_mm, s_dist_mm, 0.0f, 0.0f);
        }
    }
}

void App_SetPositionHold(bool en) {
    s_position_hold = en;
}

// x を ±limit に収める
static float Clamp(float x, float limit) {
    if (x > limit) return limit;
    if (x < -limit) return -limit;
    return x;
}

// ISRの先頭で呼ぶ。走行指令を取り込み、プロファイルを1tick進めて目標を更新する。
// 制御が無効の間は進めない(止まっているのに目標だけ進むのを防ぐ)。
static void UpdateProfile(void) {
    if (s_start_pending) {
        s_pivot_settling = false; // 合わせ込みの途中でも、新しい指令が来たらやめる
        s_front_aligning = false;
        s_profile_type = s_pending.type;
        s_profile_sign = s_pending.sign;
        s_start_pending = false;
        if (s_profile_type == MOTION_FRONT_ALIGN) {
            // その場で合わせる(止まっている所から。目標は UpdateFrontAlign が毎tick決める)
            s_profile_active = false;
            s_target_mm_s = 0.0f;
            s_target_acc = 0.0f;
            s_target_omega_dps = 0.0f;
            s_target_alpha_dps2 = 0.0f;
            s_align_ticks = 0;
            s_align_ok_ticks = 0;
            s_front_aligning = true;
            Logger_Event(LOG_EV_FRONT_ALIGN, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f); // 始め(ok = −1)
            return;
        }
        float v_start = 0.0f;
        if (s_profile_type == MOTION_STRAIGHT) {
            // 走りながら次の直進へつなげられるように(同じ向きに動いているときだけ今の速さから)
            v_start = s_profile_sign * s_target_mm_s;
            if (v_start < 0.0f) v_start = 0.0f;
            s_target_omega_dps = 0.0f;
            s_target_alpha_dps2 = 0.0f;
        } else if (s_profile_type == MOTION_PIVOT) {
            s_target_mm_s = 0.0f;        // 超信地旋回はその場で回る
            s_target_acc = 0.0f;
        } else {
            s_target_acc = 0.0f;         // スラロームは今の並進の速さのまま曲がる
        }
        VelocityProfile_StartAD(&s_profile, s_pending.distance, v_start,
                                s_pending.v_max, s_pending.v_end, s_pending.accel, s_pending.decel);
        s_profile_active = true;
    }
    if (!s_profile_active || !s_enabled) return;

    VelocityProfile_Step(&s_profile, CONTROL_DT_S);
    if (s_profile_type == MOTION_STRAIGHT) {
        s_target_mm_s = s_profile_sign * s_profile.v;
        s_target_acc = s_profile_sign * s_profile.a;
    } else {
        s_target_omega_dps = s_profile_sign * s_profile.v;
        s_target_alpha_dps2 = s_profile_sign * s_profile.a;
    }
    if (s_profile.done) {
        s_profile_active = false;
        s_target_acc = 0.0f;
        s_target_alpha_dps2 = 0.0f;
        if (s_profile_type != MOTION_STRAIGHT) s_target_omega_dps = 0.0f;
        if (s_profile_type == MOTION_PIVOT) {
            // 超信地旋回はジャイロの角度が合うまで終わりにしない(UpdatePivotSettle が終わらせる)
            s_settle_integral = 0.0f;
            s_settle_ticks = 0;
            s_settle_prev_err = 0.0f;
            s_pivot_settling = true;
        } else {
            s_motion_done = true;
            Logger_Event(LOG_EV_MOTION_DONE, (float)s_profile_type, 0.0f, 0.0f, 0.0f, 0.0f);
        }
    }
}

// 超信地旋回の合わせ込みを1tick進め、旋回の FB に足す積分の項[V]を返す(合わせ込みの間でなければ0)。
// 角度の差と角速度がどちらも小さくなるか、PIVOT_SETTLE_TIMEOUT_MS を過ぎたら、積分を捨てて終わりにする。
// 角度の差が 0 を越えたら(行き過ぎたら)積分を捨てる(2026-11-04)。積分が残ったままだと反対側へ押し続け、
// 摩擦で止まった所から戻るのに時間がかかって打ち切りになっていた(search_0014 で +1.4° → −1.2°)。
static float UpdatePivotSettle(float angle_err_rad) {
    if (!s_pivot_settling) return 0.0f;
    s_settle_ticks++;
    float err_deg = angle_err_rad * RAD_TO_DEG;
    float ms = (float)s_settle_ticks * CONTROL_DT_S * 1000.0f;
    bool ok = (fabsf(err_deg) <= PIVOT_SETTLE_TOL_DEG) && (fabsf(s_gyro_z_dps) <= PIVOT_SETTLE_OMEGA_DPS);
    if (ok || ms >= (float)PIVOT_SETTLE_TIMEOUT_MS) {
        s_pivot_settling = false;
        s_settle_integral = 0.0f;
        s_motion_done = true;
        Logger_Event(LOG_EV_PIVOT_SETTLE, ok ? 1.0f : 0.0f, err_deg, ms, 0.0f, 0.0f);
        Logger_Event(LOG_EV_MOTION_DONE, (float)MOTION_PIVOT, 0.0f, 0.0f, 0.0f, 0.0f);
        return 0.0f;
    }
    if (s_settle_prev_err * angle_err_rad < 0.0f) s_settle_integral = 0.0f; // 0 を越えた
    s_settle_prev_err = angle_err_rad;
    s_settle_integral += angle_err_rad * CONTROL_DT_S;
    float i_term = Clamp(PIVOT_SETTLE_KI * s_settle_integral, PIVOT_SETTLE_I_LIMIT_V);
    s_settle_integral = i_term / PIVOT_SETTLE_KI; // 上限に張り付いたら積分を止める
    return i_term;
}

// 前の壁で合わせるのを1tick進める(制御が有効なときだけ)。前の壁センサーから前後[mm]と向き[deg]のずれを出し、
// それを打ち消す速さと角速度を目標にする(目標の距離・目標の向きはふつうどおり目標から積むので、合わせた分も入る)。
// 両方が FRONT_ALIGN_HOLD_MS 続けて合うか、FRONT_ALIGN_TIMEOUT_MS を過ぎたら、止めて終わりにする。
static void UpdateFrontAlign(void) {
    if (!s_front_aligning || !s_enabled) return;
    s_align_ticks++;
    float fl = (float)ad_fl;
    float fr = (float)ad_fr;
    float err_mm = (fl + fr - FRONT_ALIGN_REF_SUM) / FRONT_ALIGN_SUM_PER_MM;                // + 壁に近い
    float err_deg = FRONT_ALIGN_ANGLE_ENABLE
                  ? (fl - fr - FRONT_ALIGN_REF_DIFF) / FRONT_ALIGN_DIFF_PER_DEG : 0.0f;    // + 左を向いている
    bool ok = (fabsf(err_mm) <= FRONT_ALIGN_TOL_MM) && (fabsf(err_deg) <= FRONT_ALIGN_TOL_DEG);
    s_align_ok_ticks = ok ? s_align_ok_ticks + 1u : 0u;
    float ms = (float)s_align_ticks * CONTROL_DT_S * 1000.0f;
    bool held = (float)s_align_ok_ticks * CONTROL_DT_S * 1000.0f >= (float)FRONT_ALIGN_HOLD_MS;
    if (held || ms >= (float)FRONT_ALIGN_TIMEOUT_MS) {
        s_front_aligning = false;
        s_target_mm_s = 0.0f;
        s_target_omega_dps = 0.0f;
        s_motion_done = true;
        Logger_Event(LOG_EV_FRONT_ALIGN, held ? 1.0f : 0.0f, err_mm, err_deg, ms, 0.0f);
        Logger_Event(LOG_EV_MOTION_DONE, (float)MOTION_FRONT_ALIGN, 0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }
    s_target_mm_s = Clamp(-FRONT_ALIGN_KV * err_mm, FRONT_ALIGN_V_MAX_MM_S);
    s_target_omega_dps = Clamp(-FRONT_ALIGN_KW * err_deg, FRONT_ALIGN_W_MAX_DPS);
    s_target_acc = 0.0f;
    s_target_alpha_dps2 = 0.0f;
}

// 有効化はSTBY→フラグ、無効化はフラグ→STBYの順にする。
// 無効化ではフラグを先に下ろし、ISRが新たにMotor_Drive()しないようにする。
void App_ControlLoop_SetEnabled(bool en) {
    if (en) {
        if (FailSafe_IsTripped()) return; // 発動中は有効化を拒否する
        Motor_Enable();
        s_enabled = true;
    } else {
        s_enabled = false;
        Motor_Stop();
        Motor_Disable();
    }
    Logger_Event(LOG_EV_CTRL_ENABLE, en ? 1.0f : 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

bool App_ControlLoop_IsEnabled(void) {
    return s_enabled;
}

Pose App_GetPose(void) {
    return s_odo.pose;
}

WheelVelocity App_GetActualVelocity(void) {
    return s_actual;
}

MotorPWM App_GetMotorPWM(void) {
    return s_pwm;
}

GyroData App_GetGyroRaw(void) {
    return s_gyro_raw;
}

float App_GetGyroZ_dps(void) {
    return s_gyro_z_dps;
}

float App_GetGyroAngle_deg(void) {
    return s_gyro_angle_deg;
}

void App_ResetGyroAngle(void) {
    s_gyro_angle_reset_req = true;
    uint32_t t0 = HAL_GetTick();
    while (s_gyro_angle_reset_req) {
        if (HAL_GetTick() - t0 > 10u) { // 割り込みが動いていない
            s_gyro_angle_reset_req = false;
            break;
        }
    }
}

float App_RecalibrateGyroZ(uint32_t ms) {
    uint32_t ticks = (uint32_t)((float)ms / (CONTROL_DT_S * 1000.0f));
    if (ticks == 0) ticks = 1;
    s_gyro_cal_sum = 0.0f;
    s_gyro_cal_count = 0;
    s_gyro_cal_ticks_left = ticks; // 最後に書く(ここからISRが足し込み始める)

    uint32_t t0 = HAL_GetTick();
    while (s_gyro_cal_ticks_left > 0) {
        if (HAL_GetTick() - t0 > ms * 2u + 100u) {
            s_gyro_cal_ticks_left = 0; // 割り込みが動いていない。元のゼロ点のまま
            break;
        }
        HAL_Delay(1);
    }
    return s_gyro_offset.z;
}

const ControlDebug *App_GetControlDebug(void) {
    return &s_dbg;
}

// モーター電圧[V]を、そのときの電池電圧で割ってdutyにし、
// 符号付きPWMレンジ[-PWM_MAX, PWM_MAX]へ変換・クランプする
// (PWM_MAXはinterface層の定義なのでlogic層からは参照しない)。
// Motor_Drive()側が符号でCW/CCWを切り替えるので、前後進の判断はここでは不要。
static int16_t VoltageToPWM(float volt, float vbat) {
    float pwm = volt / vbat * (float)PWM_MAX;
    if (pwm > (float)PWM_MAX) return (int16_t)PWM_MAX;
    if (pwm < -(float)PWM_MAX) return (int16_t)-PWM_MAX;
    return (int16_t)pwm;
}

// 制御していない(無効・発動中)ときのログ値。実測系は残し、出力系だけ0にする。
static void ClearOutputDebug(void) {
    s_pwm.left = 0;
    s_pwm.right = 0;
    s_dbg.pwm_l = 0.0f;
    s_dbg.pwm_r = 0.0f;
    s_dbg.ff_l = 0.0f;
    s_dbg.ff_r = 0.0f;
    s_dbg.i_s = 0.0f;
    s_dbg.fb_d = 0.0f;
    s_dbg.f_ff = 0.0f;
    s_dbg.pos_corr = 0.0f;
}

void App_ControlTick(void) {
    int16_t delta_l = Encoder_GetDeltaL();
    int16_t delta_r = Encoder_GetDeltaR();

    s_actual = Odometry_Update(&s_odo, delta_l, delta_r, CONTROL_DT_S);

    // 6軸を1回で読む。制御に使うのは今までどおりジャイロ Z だけで、加速度とジャイロ X・Y はログに残すだけ
    ImuData imu = ICM_ReadImu();
    s_gyro_raw = imu.gyro;
    if (s_gyro_cal_ticks_left > 0) {
        s_gyro_cal_sum += (float)s_gyro_raw.z;
        s_gyro_cal_count++;
        if (--s_gyro_cal_ticks_left == 0) {
            s_gyro_offset.z = s_gyro_cal_sum / (float)s_gyro_cal_count;
        }
    }
    s_gyro_z_dps = GYRO_Z_SIGN * ((float)s_gyro_raw.z - s_gyro_offset.z) / GYRO_SENSITIVITY_LSB_PER_DPS;
    {
        // 機体の座標(x 前、y 左、z 上)の物理量にする。ジャイロのゼロ点は起動時の値(Z は測り直した値)。
        // 加速度は重力を含んだまま(止まって水平なら acc_z が約 +9.8)
        ImuCalib cal = { .gyro_bias = { s_gyro_offset.x, s_gyro_offset.y, s_gyro_offset.z } };
        int16_t acc_raw[3] = { imu.accel.x, imu.accel.y, imu.accel.z };
        int16_t gyro_raw[3] = { imu.gyro.x, imu.gyro.y, imu.gyro.z };
        ImuSample body;
        Imu_ToBody(acc_raw, gyro_raw, &cal, &body);
        s_dbg.acc_x = body.acc[0];
        s_dbg.acc_y = body.acc[1];
        s_dbg.acc_z = body.acc[2];
        s_dbg.gyro_x_dps = body.gyro[0];
        s_dbg.gyro_y_dps = body.gyro[1];
    }
    if (s_gyro_angle_reset_req) {
        s_gyro_angle_deg = 0.0f;
        s_angle_ref_deg = 0.0f; // 向きの基準を決めるのはここだけ(走りの始め)
        s_gyro_angle_reset_req = false;
    }
    s_gyro_angle_deg += s_gyro_z_dps * CONTROL_DT_S;

    UpdateProfile();
    UpdateFrontAlign();

    bool control_active = s_enabled && !FailSafe_IsTripped();

    // ---- 止まっている間に、ジャイロ Z のゼロ点を測り直す ----
    // 止まれの指令(目標の速さ・角速度が 0 で、動きと合わせ込みが終わっている)の間だけ。
    // 本当に止まっているかは gyro_bias が、エンコーダ・ジャイロ・加速度の揺れで見る。新しいゼロ点は次のtickから効く
    if (GYRO_BIAS_TRACK_ENABLE) {
        bool cmd_still = control_active && s_motion_done && !s_start_pending
                      && (s_target_mm_s == 0.0f) && (s_target_omega_dps == 0.0f)
                      && !s_pivot_settling && !s_front_aligning && (s_gyro_cal_ticks_left == 0u);
        float acc_body[3] = { s_dbg.acc_x, s_dbg.acc_y, s_dbg.acc_z };
        GyroBiasStep b = GyroBias_Update(&s_bias_est, (float)s_gyro_raw.z, acc_body, delta_l, delta_r,
                                         cmd_still, s_gyro_offset.z);
        s_gyro_offset.z = b.offset;
        if (b.run_ended) {
            Logger_Event(LOG_EV_GYRO_BIAS, b.offset, b.run_change / GYRO_SENSITIVITY_LSB_PER_DPS,
                         (float)b.run_windows, 0.0f, 0.0f);
        }
    }

    // ---- 目標の向き ----
    // 基準は App_ResetGyroAngle() で決めた走りの始めの向き。制御が無効の間は進めないだけで、今の向きには合わせない。
    // ---- 目標の距離と進んだ距離 ----
    // 制御が無効の間はどちらも0にしておくので、有効にした時点が基準になる。
    if (control_active) {
        s_angle_ref_deg += s_target_omega_dps * CONTROL_DT_S;
        s_pos_ref_mm += s_target_mm_s * CONTROL_DT_S;
        s_dist_mm += 0.5f * (s_actual.left_mm_s + s_actual.right_mm_s) * CONTROL_DT_S;
        if (!s_position_hold) s_pos_ref_mm = s_dist_mm; // 位置を保たない間は、今の距離に合わせておく
        // 目標の位置は目標の速さと目標の向き、推定の位置は進んだ距離とジャイロの角度で積む
        float th_r = s_angle_ref_deg * DEG_TO_RAD;
        float th_e = s_gyro_angle_deg * DEG_TO_RAD;
        float v_e = 0.5f * (s_actual.left_mm_s + s_actual.right_mm_s);
        s_ref_x += s_target_mm_s * CONTROL_DT_S * cosf(th_r);
        s_ref_y += s_target_mm_s * CONTROL_DT_S * sinf(th_r);
        s_est_x += v_e * CONTROL_DT_S * cosf(th_e);
        s_est_y += v_e * CONTROL_DT_S * sinf(th_e);
        if (!s_position_hold) { // 位置を保たない間は、横も今の位置に合わせておく
            s_ref_x = s_est_x;
            s_ref_y = s_est_y;
        }
    } else {
        s_pos_ref_mm = 0.0f;
        s_dist_mm = 0.0f;
        s_ref_x = s_ref_y = 0.0f;
        s_est_x = s_est_y = 0.0f;
    }

    // ---- 壁切れ(横の壁がなくなった瞬間)を見つける ----
    // 直進中に、L か R が壁ありのしきい値より上から下へ切れた瞬間の目標の距離をイベントに残す(補正には使わない)。
    // 柱の横で一瞬だけ跳ねた値に引っかからないよう、その前に WALL_EDGE_MIN_WALL_MM 以上続けて壁があったときだけ数える。
    bool moving_straight = control_active && (s_target_mm_s > 0.0f) && (s_target_omega_dps == 0.0f);
    UpdateWallEdge(&s_edge_l, ad_l, WALL_TH_L, 0u, moving_straight);
    UpdateWallEdge(&s_edge_r, ad_r, WALL_TH_R, 1u, moving_straight);

    // ---- 横のずれ(目標の位置に対する推定の位置。目標の向きの左 +) ----
    // 直進している間だけ、ずれを目標の向きのずらし Δθ にして返す(それ以外は 0 へ戻す)。Δθ は速さの上限つきで動かす
    float lat_err_mm;
    {
        float th_r = s_angle_ref_deg * DEG_TO_RAD;
        float ex = s_est_x - s_ref_x;
        float ey = s_est_y - s_ref_y;
        lat_err_mm = -ex * sinf(th_r) + ey * cosf(th_r);
        s_lat_err_mm = lat_err_mm;
        s_along_err_mm = -(ex * cosf(th_r) + ey * sinf(th_r)); // 前後のずれ(+ なら目標が前)。位置のループが使う
        float ofs_target = 0.0f;
        if (LATERAL_CONTROL_ENABLE && moving_straight && !s_front_aligning) {
            ofs_target = Clamp(-lat_err_mm / LATERAL_L_MM * RAD_TO_DEG, LATERAL_OFS_MAX_DEG);
        }
        float step = LATERAL_OFS_RATE_DPS * CONTROL_DT_S;
        s_lat_ofs_deg += Clamp(ofs_target - s_lat_ofs_deg, step);
        if (!control_active) s_lat_ofs_deg = 0.0f;
    }

    // ---- 位置のループ(並進の FB の外側) ----
    // v_cmd = v_ref + POSITION_KP×(前後のずれ)。出力は並進の FB の目標になる。
    // 前後のずれは2次元(目標の位置に対する推定の位置を、目標の向きに投影した分)で見る(2026-11-04)。
    // 道に沿った距離(pos_ref − dist)だと、曲がる前の横のずれが曲がった後に前後のずれになっても見えず、
    // 小回りのたびに横と前後を行き来して消えなかった(search_0014 で 20〜25mm が約 4 秒残った)。
    float pos_corr = 0.0f;
    // 角度と同じく、止まっていても常に効かせる(止まっている間も位置を保つ)
    if (POSITION_CONTROL_ENABLE && s_position_hold && control_active) {
        pos_corr = POSITION_KP * s_along_err_mm;
        if (pos_corr > POSITION_CORR_LIMIT_MM_S) pos_corr = POSITION_CORR_LIMIT_MM_S;
        if (pos_corr < -POSITION_CORR_LIMIT_MM_S) pos_corr = -POSITION_CORR_LIMIT_MM_S;
    }

    // ---- 目標(SI) ----
    // FF はプロファイルの目標だけから計算する(2自由度制御．補正は FB が受け持つ)．
    float v_ref = s_target_mm_s * MM_TO_M;
    float a_ref = s_target_acc * MM_TO_M;
    float v_cmd = (s_target_mm_s + pos_corr) * MM_TO_M; // 並進の FB の目標(位置の補正を含む)
    float omega_ref = s_target_omega_dps * DEG_TO_RAD;
    float alpha_ref = s_target_alpha_dps2 * DEG_TO_RAD;
    float omega_meas = s_gyro_z_dps * DEG_TO_RAD;
    float v_meas = 0.5f * (s_actual.left_mm_s + s_actual.right_mm_s) * MM_TO_M;

    // 車輪速度の目標(フェイルセーフの速度偏差とログに使う．制御には使わない)
    RobotVelocity target_robot = {
        .linear_mm_s = s_target_mm_s + pos_corr,
        .angular_rad_s = omega_ref,
    };
    WheelVelocity target_wheel = Kinematics_RobotToWheel(target_robot);

    FailSafeInput fs_in = {
        .battery_v = Battery_GetVoltage(),
        .control_enabled = s_enabled,
        .target = target_wheel,
        .actual = s_actual,
        .gyro_z_dps = s_gyro_z_dps,
    };
    FailSafe_Update(&fs_in);

    float vbat = FailSafe_GetFilteredVoltage();

    s_dbg.target_mm_s = s_target_mm_s;
    s_dbg.target_acc = s_target_acc;
    s_dbg.pos_ref = s_pos_ref_mm;
    s_dbg.dist_mm = s_dist_mm;
    s_dbg.pos_corr = pos_corr;
    s_dbg.x_mm = s_odo.pose.x_mm;
    s_dbg.vl = s_actual.left_mm_s;
    s_dbg.vr = s_actual.right_mm_s;
    s_dbg.vl_ref = target_wheel.left_mm_s;
    s_dbg.vr_ref = target_wheel.right_mm_s;
    s_dbg.vbat = vbat;
    s_dbg.target_omega_dps = s_target_omega_dps;
    s_dbg.gyro_z_dps = s_gyro_z_dps;
    s_dbg.angle_deg = s_gyro_angle_deg;
    s_dbg.angle_ref_deg = s_angle_ref_deg;
    s_dbg.lat_err_mm = lat_err_mm;
    // 壁センサー(このtickの先頭で Sensor_ReadAll() が更新した値)。制御が無効の間も記録する
    s_dbg.ad_l = (float)ad_l;
    s_dbg.ad_fl = (float)ad_fl;
    s_dbg.ad_fr = (float)ad_fr;
    s_dbg.ad_r = (float)ad_r;

    // 発動中は毎tick止め直す。メインが直前にSetEnabled(true)と競合して
    // STBYをHighにしていても、ここで必ずLowへ戻る。
    if (FailSafe_IsTripped()) {
        s_enabled = false;
        Motor_Stop();
        Motor_Disable();
        ClearOutputDebug();
        s_prev_enabled = false;
        // 走行中の指令も打ち切る(再開しないので、完了扱いにして待ちを抜けさせる)
        s_start_pending = false;
        s_profile_active = false;
        s_pivot_settling = false;
        s_front_aligning = false;
        s_motion_done = true;
        s_target_mm_s = 0.0f;
        s_target_acc = 0.0f;
        s_target_omega_dps = 0.0f;
        s_target_alpha_dps2 = 0.0f;
        return;
    }

    if (!s_enabled) {
        Motor_Stop();
        ClearOutputDebug();
        s_prev_enabled = false;
        return;
    }
    // 有効化の立ち上がりで，並進の積分と F の FF を捨てる．
    if (!s_prev_enabled) {
        s_vs_integral = 0.0f;
        DriveModel_Reset(&s_model);
    }
    s_prev_enabled = true;

    // ---- FF(モデルの逆) ----
    float vs_ff = DriveModel_FF_Vs(v_ref, a_ref, omega_ref);
    float vd_ff = DriveModel_FF_Vd(&s_model, v_ref, omega_ref, alpha_ref, CONTROL_DT_S);

    // ---- 並進の FB(PI) ----
    float e_v = v_cmd - v_meas;
    s_vs_integral += e_v * CONTROL_DT_S;
    float i_term = Clamp(DRIVE_VS_KI * s_vs_integral, DRIVE_VS_I_LIMIT_V);
    s_vs_integral = i_term / DRIVE_VS_KI; // 上限に張り付いたら積分を止める
    float vs_fb = Clamp(DRIVE_VS_KP * e_v + i_term, DRIVE_VS_FB_LIMIT_V);

    // ---- 旋回の FB(PD．超信地旋回の合わせ込みの間だけ積分を足す) ----
    // 制御が有効な間は，止まっていても常に効かせる(止まっている間も向きを保つ)．
    float vd_fb = 0.0f;
    float angle_err_rad = (s_angle_ref_deg + s_lat_ofs_deg - s_gyro_angle_deg) * DEG_TO_RAD; // 横のずれの分を足した向き
    float vd_settle_i = UpdatePivotSettle(angle_err_rad); // FB を切っていても，合わせ込みは打ち切りで終わる
    if (ANGULAR_CONTROL_ENABLE) {
        vd_fb = Clamp(DRIVE_VD_K_OMEGA * (omega_ref - omega_meas) + DRIVE_VD_K_THETA * angle_err_rad
                      + vd_settle_i, DRIVE_VD_FB_LIMIT_V);
    }

    float vs = vs_ff + vs_fb;
    float vd = vd_ff + vd_fb;

    if (vbat < VELOCITY_VBAT_MIN_V) vbat = VELOCITY_VBAT_MIN_V;

    s_pwm.left  = VoltageToPWM(vs - vd, vbat);
    s_pwm.right = VoltageToPWM(vs + vd, vbat);

    s_dbg.pwm_l = (float)s_pwm.left;
    s_dbg.pwm_r = (float)s_pwm.right;
    s_dbg.ff_l = vs_ff - vd_ff;
    s_dbg.ff_r = vs_ff + vd_ff;
    s_dbg.i_s = i_term;
    s_dbg.fb_d = vd_fb;
    s_dbg.f_ff = s_model.f_ff;

    Motor_Drive(s_pwm.left, s_pwm.right);
}
