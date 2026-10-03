#include "app/control_loop.h"

#include <math.h>
#include "interface/encoder.h"
#include "interface/motor.h"
#include "interface/gyro.h"
#include "interface/battery.h"
#include "logic/state_estimation/kinematics.h"
#include "logic/control/velocity_pid.h"
#include "logic/control/velocity_profile.h"
#include "app/failsafe.h"

static Odometry_t s_odo;
static VelocityPID s_vpid;
static WheelVelocity s_actual;
static float s_target_mm_s = 0.0f;
static float s_target_acc = 0.0f;
static MotorPWM s_pwm = { 0, 0 };

// 直進プロファイル。ISRだけが進める。
static VelocityProfile s_profile;
static volatile bool s_profile_active = false;
static volatile bool s_motion_done = true;

// メイン→ISRへの直進指令の受け渡し。パラメータを書いてから最後にフラグを立て、
// ISRは次のtickでフラグを見て取り込む(取り込み中にメインが書き換えることはない)。
typedef struct {
    float distance_mm;
    float v_max;
    float v_end;
    float accel;
} StraightCommand;
static StraightCommand s_pending;
static volatile bool s_start_pending = false;
static GyroData s_gyro_raw = { 0, 0, 0 };
static float s_gyro_z_dps = 0.0f;
static GyroOffset s_gyro_offset = { 0.0f, 0.0f, 0.0f };
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
    s_target_mm_s = mm_s;
}

void App_StartStraight(float distance_mm, float v_max, float v_end, float accel) {
    s_pending.distance_mm = distance_mm;
    s_pending.v_max = v_max;
    s_pending.v_end = v_end;
    s_pending.accel = accel;
    s_motion_done = false;
    s_start_pending = true; // 最後に立てる
}

bool App_IsMotionDone(void) {
    return s_motion_done && !s_start_pending;
}

// ISRの先頭で呼ぶ。直進指令を取り込み、プロファイルを1tick進めて目標を更新する。
// 制御が無効の間は進めない(止まっているのに目標だけ進むのを防ぐ)。
static void UpdateProfile(void) {
    if (s_start_pending) {
        VelocityProfile_Start(&s_profile, s_pending.distance_mm, s_target_mm_s,
                              s_pending.v_max, s_pending.v_end, s_pending.accel);
        s_profile_active = true;
        s_start_pending = false;
    }
    if (!s_profile_active || !s_enabled) return;

    VelocityProfile_Step(&s_profile, CONTROL_DT_S);
    s_target_mm_s = s_profile.v;
    s_target_acc = s_profile.a;
    if (s_profile.done) {
        s_profile_active = false;
        s_target_acc = 0.0f;
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
}

void App_ControlTick(void) {
    int16_t delta_l = Encoder_GetDeltaL();
    int16_t delta_r = Encoder_GetDeltaR();

    s_actual = Odometry_Update(&s_odo, delta_l, delta_r, CONTROL_DT_S);

    s_gyro_raw = ICM_ReadGyro();
    s_gyro_z_dps = GYRO_Z_SIGN * ((float)s_gyro_raw.z - s_gyro_offset.z) / GYRO_SENSITIVITY_LSB_PER_DPS;

    UpdateProfile();

    RobotVelocity target_robot = { .linear_mm_s = s_target_mm_s, .angular_rad_s = 0.0f };
    WheelVelocity target_wheel = Kinematics_RobotToWheel(target_robot);
    // 加速度も速度と同じ線形変換で車輪ごとに分ける(直進なので左右同じ)。
    RobotVelocity target_robot_acc = { .linear_mm_s = s_target_acc, .angular_rad_s = 0.0f };
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
    s_dbg.pos_ref = s_profile.pos_mm;
    s_dbg.x_mm = s_odo.pose.x_mm;
    s_dbg.vl = s_actual.left_mm_s;
    s_dbg.vr = s_actual.right_mm_s;
    s_dbg.vbat = vbat;

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
        VelocityPID_Reset(&s_vpid);
        s_prev_enabled = true;
    }

    // 2自由度制御: FFは目標(速度・加速度)だけから、PIDは偏差から。単位はどちらも電圧[V]。
    float ff_l = VELOCITY_FF_FRIC_L * SignOf(target_wheel.left_mm_s)
               + VELOCITY_FF_GAIN_L * target_wheel.left_mm_s
               + VELOCITY_FF_ACC_L  * target_wheel_acc.left_mm_s;
    float ff_r = VELOCITY_FF_FRIC_R * SignOf(target_wheel.right_mm_s)
               + VELOCITY_FF_GAIN_R * target_wheel.right_mm_s
               + VELOCITY_FF_ACC_R  * target_wheel_acc.right_mm_s;

    WheelVelocity out;
    bool stopped = (s_target_mm_s == 0.0f) && (s_target_acc == 0.0f)
                && (fabsf(s_actual.left_mm_s)  < VELOCITY_STOP_RESET_MM_S)
                && (fabsf(s_actual.right_mm_s) < VELOCITY_STOP_RESET_MM_S);
    if (stopped) {
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
