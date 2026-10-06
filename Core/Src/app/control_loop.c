#include "app/control_loop.h"

#include <math.h>
#include "main.h" // HAL_GetTick / HAL_Delay(App_RecalibrateGyroZ の待ち)
#include "interface/encoder.h"
#include "interface/motor.h"
#include "interface/gyro.h"
#include "interface/battery.h"
#include "logic/state_estimation/kinematics.h"
#include "logic/control/velocity_pid.h"
#include "logic/control/velocity_profile.h"
#include "app/failsafe.h"

#define DEG_TO_RAD (3.14159265f / 180.0f)

static Odometry_t s_odo;
static VelocityPID s_vpid;
static WheelVelocity s_actual;

// 角度・角速度のループ(外側)。ISRからのみ触る。
// 目標の向き[deg]。制御が無効の間は今の向きに合わせておき、有効にした時点の向きを基準に
// プロファイルの角速度を積分して進める。止まってもリセットしない。
static float s_angle_ref_deg = 0.0f;

// 位置のループ(外側、並進方向)。ISRからのみ触る。
// 目標の距離と進んだ距離[mm]。どちらも制御が無効の間は0にしておき、有効にした時点を基準にする。
static float s_pos_ref_mm = 0.0f;
static float s_dist_mm = 0.0f;
static volatile bool s_position_hold = true; // メインが書き、ISRが読む

// 目標(並進・回転)。プロファイル実行中はISRが毎tick書き換える。
static float s_target_mm_s = 0.0f;
static float s_target_acc = 0.0f;
static float s_target_omega_dps = 0.0f;
static float s_target_alpha_dps2 = 0.0f;
static MotorPWM s_pwm = { 0, 0 };

// 走行プロファイル(直進 or 超信地旋回の一方)。ISRだけが進める。
// 超信地旋回は、角度[deg]を「距離」、角速度[dps]を「速度」として同じ台形を使う。
typedef enum {
    MOTION_STRAIGHT,
    MOTION_PIVOT,
} MotionType;

static VelocityProfile s_profile;
static MotionType s_profile_type = MOTION_STRAIGHT;
static float s_profile_sign = 1.0f; // 超信地旋回の向き(+1: 反時計回り)
static volatile bool s_profile_active = false;
static volatile bool s_motion_done = true;

// メイン→ISRへの走行指令の受け渡し。パラメータを書いてから最後にフラグを立て、
// ISRは次のtickでフラグを見て取り込む(取り込み中にメインが書き換えることはない)。
typedef struct {
    MotionType type;
    float distance; // [mm] or [deg](正の値)
    float v_max;    // [mm/s] or [dps]
    float v_end;    // [mm/s](超信地旋回は0)
    float accel;    // [mm/s^2] or [dps^2]
    float sign;     // 超信地旋回の向き
} MotionCommand;
static MotionCommand s_pending;
static volatile bool s_start_pending = false;

static GyroData s_gyro_raw = { 0, 0, 0 };
static float s_gyro_z_dps = 0.0f;
static float s_gyro_angle_deg = 0.0f; // ジャイロの積分角(起動からの累積、反時計回り正)
static GyroOffset s_gyro_offset = { 0.0f, 0.0f, 0.0f };

// ジャイロ Z のゼロ点の測り直し。メインが合計と回数を0にしてから残りtick数を書き、
// ISRが毎tick足し込んで、0になったらゼロ点を更新する。
static volatile uint32_t s_gyro_cal_ticks_left = 0;
static float s_gyro_cal_sum = 0.0f;
static uint32_t s_gyro_cal_count = 0;
static ControlDebug s_dbg;

// メインループが書き、ISRが読む。フェイルセーフ発動時はISRもfalseを書く。
static volatile bool s_enabled = false;
static bool s_prev_enabled = false;     // ISRからのみ触る

void App_ControlLoop_Init(void) {
    Odometry_Reset(&s_odo);
    VelocityPID_Init(&s_vpid);
    FailSafe_Init();
    s_gyro_offset = ICM_GetOffset(); // ICM_CalibrateBlocking()後に呼ばれる前提
    s_actual.left_mm_s = 0.0f;
    s_actual.right_mm_s = 0.0f;
}

void App_SetTargetVelocity(float mm_s) {
    // プロファイルを先に止めてから目標を書く(ISRが上書きしないように)。
    s_start_pending = false;
    s_profile_active = false;
    s_motion_done = true;
    s_target_acc = 0.0f;
    s_target_omega_dps = 0.0f;
    s_target_alpha_dps2 = 0.0f;
    s_target_mm_s = mm_s;
}

void App_StartStraight(float distance_mm, float v_max, float v_end, float accel) {
    s_pending.type = MOTION_STRAIGHT;
    s_pending.distance = distance_mm;
    s_pending.v_max = v_max;
    s_pending.v_end = v_end;
    s_pending.accel = accel;
    s_pending.sign = 1.0f;
    s_motion_done = false;
    s_start_pending = true; // 最後に立てる
}

void App_StartPivot(float angle_deg, float omega_max_dps, float alpha_dps2) {
    s_pending.type = MOTION_PIVOT;
    s_pending.distance = fabsf(angle_deg);
    s_pending.v_max = omega_max_dps;
    s_pending.v_end = 0.0f;
    s_pending.accel = alpha_dps2;
    s_pending.sign = (angle_deg >= 0.0f) ? 1.0f : -1.0f;
    s_motion_done = false;
    s_start_pending = true; // 最後に立てる
}

bool App_IsMotionDone(void) {
    return s_motion_done && !s_start_pending;
}

void App_SetPositionHold(bool en) {
    s_position_hold = en;
}

// ISRの先頭で呼ぶ。走行指令を取り込み、プロファイルを1tick進めて目標を更新する。
// 制御が無効の間は進めない(止まっているのに目標だけ進むのを防ぐ)。
static void UpdateProfile(void) {
    if (s_start_pending) {
        s_profile_type = s_pending.type;
        s_profile_sign = s_pending.sign;
        float v_start = 0.0f;
        if (s_profile_type == MOTION_STRAIGHT) {
            v_start = s_target_mm_s;     // 走りながら次の直進へつなげられるように
            s_target_omega_dps = 0.0f;
            s_target_alpha_dps2 = 0.0f;
        } else {
            s_target_mm_s = 0.0f;        // 超信地旋回はその場で回る
            s_target_acc = 0.0f;
        }
        VelocityProfile_Start(&s_profile, s_pending.distance, v_start,
                              s_pending.v_max, s_pending.v_end, s_pending.accel);
        s_profile_active = true;
        s_start_pending = false;
    }
    if (!s_profile_active || !s_enabled) return;

    VelocityProfile_Step(&s_profile, CONTROL_DT_S);
    if (s_profile_type == MOTION_STRAIGHT) {
        s_target_mm_s = s_profile.v;
        s_target_acc = s_profile.a;
    } else {
        s_target_omega_dps = s_profile_sign * s_profile.v;
        s_target_alpha_dps2 = s_profile_sign * s_profile.a;
    }
    if (s_profile.done) {
        s_profile_active = false;
        s_target_acc = 0.0f;
        s_target_alpha_dps2 = 0.0f;
        if (s_profile_type == MOTION_PIVOT) s_target_omega_dps = 0.0f;
        s_motion_done = true;
    }
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

// 摩擦FF用の符号。目標0のときは0(止まっている所に摩擦ぶんの電圧をかけない)。
static float SignOf(float v) {
    if (v > 0.0f) return 1.0f;
    if (v < 0.0f) return -1.0f;
    return 0.0f;
}

// 制御していない(無効・発動中)ときのログ値。実測系は残し、出力系だけ0にする。
static void ClearOutputDebug(void) {
    s_pwm.left = 0;
    s_pwm.right = 0;
    s_dbg.pwm_l = 0.0f;
    s_dbg.pwm_r = 0.0f;
    s_dbg.ff_l = 0.0f;
    s_dbg.ff_r = 0.0f;
    s_dbg.i_l = 0.0f;
    s_dbg.i_r = 0.0f;
    s_dbg.ang_corr_dps = 0.0f;
    s_dbg.pos_corr = 0.0f;
}

void App_ControlTick(void) {
    int16_t delta_l = Encoder_GetDeltaL();
    int16_t delta_r = Encoder_GetDeltaR();

    s_actual = Odometry_Update(&s_odo, delta_l, delta_r, CONTROL_DT_S);

    s_gyro_raw = ICM_ReadGyro();
    if (s_gyro_cal_ticks_left > 0) {
        s_gyro_cal_sum += (float)s_gyro_raw.z;
        s_gyro_cal_count++;
        if (--s_gyro_cal_ticks_left == 0) {
            s_gyro_offset.z = s_gyro_cal_sum / (float)s_gyro_cal_count;
        }
    }
    s_gyro_z_dps = GYRO_Z_SIGN * ((float)s_gyro_raw.z - s_gyro_offset.z) / GYRO_SENSITIVITY_LSB_PER_DPS;
    s_gyro_angle_deg += s_gyro_z_dps * CONTROL_DT_S;

    UpdateProfile();

    bool control_active = s_enabled && !FailSafe_IsTripped();

    // ---- 目標の向き ----
    // 制御が無効の間は今の向きに合わせておくので、有効にした時点の向きが基準になる。
    // ---- 目標の距離と進んだ距離 ----
    // 制御が無効の間はどちらも0にしておくので、有効にした時点が基準になる。
    if (control_active) {
        s_angle_ref_deg += s_target_omega_dps * CONTROL_DT_S;
        s_pos_ref_mm += s_target_mm_s * CONTROL_DT_S;
        s_dist_mm += 0.5f * (s_actual.left_mm_s + s_actual.right_mm_s) * CONTROL_DT_S;
        if (!s_position_hold) s_pos_ref_mm = s_dist_mm; // 位置を保たない間は、今の距離に合わせておく
    } else {
        s_angle_ref_deg = s_gyro_angle_deg;
        s_pos_ref_mm = 0.0f;
        s_dist_mm = 0.0f;
    }

    // ---- 位置のループ(外側、並進方向) ----
    // v_cmd = v_ref + POSITION_KP×(s_ref − s)。出力は車輪速度の目標になる。
    float pos_corr = 0.0f;
    // 角度と同じく、止まっていても常に効かせる(止まっている間も位置を保つ)
    if (POSITION_CONTROL_ENABLE && s_position_hold && control_active) {
        pos_corr = POSITION_KP * (s_pos_ref_mm - s_dist_mm);
        if (pos_corr > POSITION_CORR_LIMIT_MM_S) pos_corr = POSITION_CORR_LIMIT_MM_S;
        if (pos_corr < -POSITION_CORR_LIMIT_MM_S) pos_corr = -POSITION_CORR_LIMIT_MM_S;
    }

    // ---- 角度・角速度のループ(外側) ----
    // ω_cmd = ω_ref + ANGLE_KP×(θ_ref − θ) + ANGULAR_KP×(ω_ref − ω)。出力は車輪速度の目標になる。
    float omega_ref = s_target_omega_dps * DEG_TO_RAD;
    float omega_meas = s_gyro_z_dps * DEG_TO_RAD;
    float angle_err_deg = s_angle_ref_deg - s_gyro_angle_deg;
    float ang_corr = 0.0f;
    // 制御が有効な間は、止まっていても常に効かせる(止まっている間も向きを保つ)
    if (ANGULAR_CONTROL_ENABLE && control_active) {
        ang_corr = ANGLE_KP * angle_err_deg * DEG_TO_RAD + ANGULAR_KP * (omega_ref - omega_meas);
        if (ang_corr > ANGULAR_CORR_LIMIT_RAD_S) ang_corr = ANGULAR_CORR_LIMIT_RAD_S;
        if (ang_corr < -ANGULAR_CORR_LIMIT_RAD_S) ang_corr = -ANGULAR_CORR_LIMIT_RAD_S;
    }

    // 止まるべき所で止まったら、車輪速度ループの I 項を捨てて出力0にする(摩擦で止まった後に
    // PWM が出続けるのを防ぐ)。角度か位置の制御が有効なら、止まっている間も保つので使わない。
    bool robot_stopped = !ANGULAR_CONTROL_ENABLE && !POSITION_CONTROL_ENABLE
                      && (s_target_mm_s == 0.0f) && (s_target_acc == 0.0f)
                      && (s_target_omega_dps == 0.0f) && (s_target_alpha_dps2 == 0.0f)
                      && (fabsf(s_actual.left_mm_s)  < VELOCITY_STOP_RESET_MM_S)
                      && (fabsf(s_actual.right_mm_s) < VELOCITY_STOP_RESET_MM_S);

    RobotVelocity target_robot = {
        .linear_mm_s = s_target_mm_s + pos_corr,
        .angular_rad_s = omega_ref + ang_corr,
    };
    WheelVelocity target_wheel = Kinematics_RobotToWheel(target_robot);
    // FF はプロファイルの目標だけから計算する(2自由度制御。補正は PID が受け持つ)。
    // 補正を含めると、止まっていて補正がほぼ0のとき、摩擦FFの符号が誤差の揺れで反転してしまう。
    RobotVelocity target_robot_ff = {
        .linear_mm_s = s_target_mm_s,
        .angular_rad_s = omega_ref,
    };
    WheelVelocity target_wheel_ff = Kinematics_RobotToWheel(target_robot_ff);
    // 加速度も速度と同じ線形変換で車輪ごとに分ける(FFには目標の加速度だけを使う)。
    RobotVelocity target_robot_acc = {
        .linear_mm_s = s_target_acc,
        .angular_rad_s = s_target_alpha_dps2 * DEG_TO_RAD,
    };
    WheelVelocity target_wheel_acc = Kinematics_RobotToWheel(target_robot_acc);

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
    s_dbg.ang_corr_dps = ang_corr / DEG_TO_RAD;

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
    if (!s_prev_enabled) {
        // 有効化の立ち上がり。無効中に溜まった積分項・前回偏差を捨てる。
        // (目標の向きは、無効の間ずっと今の向きに合わせてあるので、ここでは何もしない)
        VelocityPID_Reset(&s_vpid);
        s_prev_enabled = true;
    }

    // ---- 車輪速度ループ(内側) ----
    // 2自由度制御: FFは目標(速度・加速度)だけから、PIDは偏差から。単位はどちらも電圧[V]。
    float ff_l = VELOCITY_FF_FRIC_L * SignOf(target_wheel_ff.left_mm_s)
               + VELOCITY_FF_GAIN_L * target_wheel_ff.left_mm_s
               + VELOCITY_FF_ACC_L  * target_wheel_acc.left_mm_s;
    float ff_r = VELOCITY_FF_FRIC_R * SignOf(target_wheel_ff.right_mm_s)
               + VELOCITY_FF_GAIN_R * target_wheel_ff.right_mm_s
               + VELOCITY_FF_ACC_R  * target_wheel_acc.right_mm_s;

    // 超信地旋回(直進の目標が0で、旋回の目標がある)では、タイヤが横にこすれる摩擦を足す。
    if (s_target_mm_s == 0.0f && s_target_acc == 0.0f && s_target_omega_dps != 0.0f) {
        // 回る向きで必要な電圧が違うので、向きごとに分ける
        if (s_target_omega_dps > 0.0f) {
            // 反時計回り(左回り): 右の車輪が前へ、左が後ろへ
            ff_l -= PIVOT_FF_FRIC_CCW_L;
            ff_r += PIVOT_FF_FRIC_CCW_R;
        } else {
            // 時計回り(右回り): 左の車輪が前へ、右が後ろへ
            ff_l += PIVOT_FF_FRIC_CW_L;
            ff_r -= PIVOT_FF_FRIC_CW_R;
        }
    }

    WheelVelocity out;
    if (robot_stopped) {
        // 止まるべき所で止まった。I項を捨てて出力0にし、PWMが残り続けるのを防ぐ。
        VelocityPID_Reset(&s_vpid);
        out.left_mm_s = 0.0f;
        out.right_mm_s = 0.0f;
    } else {
        out = VelocityPID_Update(&s_vpid, target_wheel, s_actual, CONTROL_DT_S);
    }

    if (vbat < VELOCITY_VBAT_MIN_V) vbat = VELOCITY_VBAT_MIN_V;

    s_pwm.left  = VoltageToPWM(ff_l + out.left_mm_s,  vbat);
    s_pwm.right = VoltageToPWM(ff_r + out.right_mm_s, vbat);

    s_dbg.pwm_l = (float)s_pwm.left;
    s_dbg.pwm_r = (float)s_pwm.right;
    s_dbg.ff_l = ff_l;
    s_dbg.ff_r = ff_r;
    s_dbg.i_l = s_vpid.left.ki  * s_vpid.left.integral;
    s_dbg.i_r = s_vpid.right.ki * s_vpid.right.integral;

    Motor_Drive(s_pwm.left, s_pwm.right);
}
