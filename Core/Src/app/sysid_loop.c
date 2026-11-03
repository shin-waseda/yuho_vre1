#include "app/sysid_loop.h"

#include <math.h>
#include "main.h" // HAL_GetTick / HAL_Delay(SysidLoop_RecalibrateImu の待ち)
#include "interface/encoder.h"
#include "interface/motor.h"
#include "interface/gyro.h"
#include "interface/battery.h"
#include "app/failsafe.h"
#include "app/logger.h"

static SysidDebug s_dbg;

// 走行。メインが s_cfg を書いてから s_start_req を立て、ISR が次の tick で取り込む。
static StepConfig s_cfg;
static volatile bool s_start_req = false;
static SysidLogStart s_log_start = SYSID_LOG_START; // 記録の始め方(E1S は続きから・走る前から)
static volatile bool s_running = false;
static volatile bool s_done = true;
static StepTest s_step;      // ISR からのみ触る
static SysidResult s_result; // ISR が書き、s_done の後にメインが読む
// FF の検証(E1V)。s_is_ff なら step_test の代わりに ff_test で走る
static FfConfig s_ff_cfg;
static FfTest s_ff;          // ISR からのみ触る
static bool s_is_ff = false;

// 走り始めからの左右の車輪の進んだ距離[mm]
static float s_dist_l_mm = 0.0f;
static float s_dist_r_mm = 0.0f;

static float s_gyro_z_dps = 0.0f;
static float s_gyro_angle_deg = 0.0f;
static ImuCalib s_imu_cal; // ジャイロのゼロ点・止まっているときの加速度(センサーの軸、生値)

// 6軸の止まっているときの値の測り直し。メインが足し込みを空にしてから残りtick数を書き、
// ISRが毎tick足し込んで、0になったら s_imu_cal を更新する。
static volatile uint32_t s_imu_cal_ticks_left = 0;
static ImuCalAcc s_imu_cal_acc;

void SysidLoop_Init(void) {
    FailSafe_Init();
    GyroOffset off = ICM_GetOffset(); // ICM_CalibrateBlocking()後に呼ばれる前提
    s_imu_cal.gyro_bias[0] = off.x;
    s_imu_cal.gyro_bias[1] = off.y;
    s_imu_cal.gyro_bias[2] = off.z;
    for (int i = 0; i < 3; i++) s_imu_cal.acc_rest[i] = 0.0f; // 最初の測り直しまでは 0
    s_dbg.seg = -1.0f;
}

void SysidLoop_StartRun(const StepConfig *cfg, SysidLogStart log) {
    if (s_running || s_start_req) return;
    if (FailSafe_IsTripped()) return;
    s_cfg = *cfg;
    s_is_ff = false;
    s_log_start = log;
    s_done = false;
    Motor_Enable();
    s_start_req = true; // 最後に立てる
}

void SysidLoop_StartFfRun(const FfConfig *cfg) {
    if (s_running || s_start_req) return;
    if (FailSafe_IsTripped()) return;
    s_ff_cfg = *cfg;
    s_is_ff = true;
    s_log_start = SYSID_LOG_START;
    s_done = false;
    Motor_Enable();
    s_start_req = true; // 最後に立てる
}

bool SysidLoop_IsDone(void) {
    return s_done;
}

SysidResult SysidLoop_GetResult(void) {
    return s_result;
}

float SysidLoop_GetVbat(void) {
    return FailSafe_GetFilteredVoltage();
}

const SysidDebug *SysidLoop_GetDebug(void) {
    return &s_dbg;
}

ImuCalib SysidLoop_RecalibrateImu(uint32_t ms) {
    uint32_t ticks = (uint32_t)((float)ms / (CONTROL_DT_S * 1000.0f));
    if (ticks == 0) ticks = 1;
    Imu_CalReset(&s_imu_cal_acc);
    s_imu_cal_ticks_left = ticks; // 最後に書く(ここからISRが足し込み始める)

    uint32_t t0 = HAL_GetTick();
    while (s_imu_cal_ticks_left > 0) {
        if (HAL_GetTick() - t0 > ms * 2u + 100u) {
            s_imu_cal_ticks_left = 0; // 割り込みが動いていない。元の値のまま
            break;
        }
        Logger_StreamPoll(); // 流していなければ何もしない
        HAL_Delay(1);
    }
    return s_imu_cal;
}

// 電圧[V]を、そのときの電池の電圧で割って duty にし、符号付きの PWM[-PWM_MAX, PWM_MAX] にする(四捨五入)。
// 範囲を超えたら上限で止め、*saturated を true にする。
static int16_t VoltageToPWM(float volt, float vbat, bool *saturated) {
    float pwm = volt / vbat * (float)PWM_MAX;
    if (pwm > (float)PWM_MAX) {
        *saturated = true;
        return (int16_t)PWM_MAX;
    }
    if (pwm < -(float)PWM_MAX) {
        *saturated = true;
        return (int16_t)-PWM_MAX;
    }
    return (int16_t)lroundf(pwm);
}

// 走り終わった(フェイルセーフを含む)。モーターを止め、記録を止める
static void FinishRun(bool failsafe) {
    Motor_Stop();
    Motor_Disable();
    s_dbg.pwm_l = 0.0f;
    s_dbg.pwm_r = 0.0f;
    s_dbg.v_cmd = 0.0f;
    s_dbg.v_ref = 0.0f;
    s_dbg.a_ref = 0.0f;
    s_dbg.v_s = 0.0f;
    s_dbg.v_ff = 0.0f;
    s_dbg.v_p = 0.0f;
    s_dbg.v_i = 0.0f;
    s_dbg.v_d = 0.0f;
    s_dbg.v_d0 = 0.0f;
    s_dbg.dv = 0.0f;
    s_result.failsafe = failsafe;
    s_result.dist_mm = s_dbg.dist;
    s_result.angle_deg = s_gyro_angle_deg;
    if (s_is_ff) {
        s_result.abort = s_ff.abort;
        s_result.v_d_meas = s_ff.v_d_meas;
        s_result.step = s_ff.cfg.step_no;
        s_result.steps = 1u;
    } else {
        s_result.abort = s_step.abort;
        s_result.moved_l = s_step.moved_l;
        s_result.moved_r = s_step.moved_r;
        s_result.move_v_l = s_step.move_v_l;
        s_result.move_v_r = s_step.move_v_r;
        s_result.step = (uint16_t)(s_step.step + 1u);
        s_result.steps = s_step.steps;
    }
    Logger_Stop();
    s_running = false;
    s_dbg.seg = -1.0f;
    s_dbg.step = 0.0f;
    s_done = true; // 最後に立てる(メインはこれを見て結果を読む)
}

void SysidLoop_Tick(void) {
    // ---- 測る ----
    int16_t delta_l = Encoder_GetDeltaL();
    int16_t delta_r = Encoder_GetDeltaR();
    float dl = ENCODER_L_SIGN * (float)delta_l * MM_PER_PULSE;
    float dr = ENCODER_R_SIGN * (float)delta_r * MM_PER_PULSE;
    s_dbg.vl = dl / CONTROL_DT_S;
    s_dbg.vr = dr / CONTROL_DT_S;

    ImuData imu = ICM_ReadImu();
    int16_t acc_raw[3] = { imu.accel.x, imu.accel.y, imu.accel.z };
    int16_t gyro_raw[3] = { imu.gyro.x, imu.gyro.y, imu.gyro.z };
    if (s_imu_cal_ticks_left > 0) {
        Imu_CalAdd(&s_imu_cal_acc, acc_raw, gyro_raw);
        if (--s_imu_cal_ticks_left == 0) Imu_CalFinish(&s_imu_cal_acc, &s_imu_cal);
    }
    ImuSample imu_s;
    Imu_ToBody(acc_raw, gyro_raw, &s_imu_cal, &imu_s);
    s_gyro_z_dps = imu_s.gyro[2];
    s_dbg.gyro_x = imu_s.gyro[0];
    s_dbg.gyro_y = imu_s.gyro[1];
    s_dbg.acc_x = imu_s.acc[0];
    s_dbg.acc_y = imu_s.acc[1];
    s_dbg.acc_z = imu_s.acc[2];

    FailSafeInput fs_in = {
        .battery_v = Battery_GetVoltage(),
        .gyro_z_dps = s_gyro_z_dps,
    };
    FailSafe_Update(&fs_in);
    float vbat = FailSafe_GetFilteredVoltage();

    // ---- 走り始め: 距離・積分角を 0 にし、step_test と記録を同じ tick から始める ----
    if (s_start_req) {
        s_dist_l_mm = 0.0f;
        s_dist_r_mm = 0.0f;
        s_gyro_angle_deg = 0.0f;
        s_result = (SysidResult){ 0 };
        s_result.vbat_start = vbat;
        if (s_is_ff) {
            FfTest_Start(&s_ff, &s_ff_cfg, CONTROL_DT_S);
        } else {
            StepTest_Start(&s_step, &s_cfg, CONTROL_DT_S);
        }
        s_running = true;
        s_start_req = false;
        // このtickの終わりの Logger_Sample から記録する(KEEP はもう記録している)
        if (s_log_start == SYSID_LOG_RESUME) {
            Logger_Resume();
        } else if (s_log_start == SYSID_LOG_START) {
            Logger_Start();
        }
    } else {
        s_dist_l_mm += dl;
        s_dist_r_mm += dr;
        s_gyro_angle_deg += s_gyro_z_dps * CONTROL_DT_S;
    }

    s_dbg.gyro_z = s_gyro_z_dps;
    s_dbg.angle = s_gyro_angle_deg;
    s_dbg.dist = 0.5f * (s_dist_l_mm + s_dist_r_mm);
    s_dbg.vbat = vbat;

    if (FailSafe_IsTripped()) {
        if (s_running) {
            FinishRun(true);
        } else {
            Motor_Stop();
            Motor_Disable();
        }
        return;
    }
    if (!s_running) return;

    // ---- 走る ----
    StepMeas m = {
        .dist_l_mm = s_dist_l_mm,
        .dist_r_mm = s_dist_r_mm,
        .vl_mm_s = s_dbg.vl,
        .vr_mm_s = s_dbg.vr,
        .v_mm_s = 0.5f * (s_dbg.vl + s_dbg.vr),
        .omega_dps = s_gyro_z_dps,
        .angle_deg = s_gyro_angle_deg,
        .vbat_v = vbat,
        .front_ir = (float)((ad_fl > ad_fr) ? ad_fl : ad_fr), // Sensor_ReadAll(同じ割り込みの前)が書いた値
    };
    float v_left, v_right;
    if (s_is_ff) {
        FfTest_Update(&s_ff, &m);
        if (s_ff.done) {
            FinishRun(false);
            return;
        }
        v_left = s_ff.v_left;
        v_right = s_ff.v_right;
    } else {
        StepTest_Update(&s_step, &m);
        if (s_step.done) {
            FinishRun(false);
            return;
        }
        v_left = s_step.v_left;
        v_right = s_step.v_right;
    }

    float v_abs = fabsf(m.v_mm_s);
    float w_abs = fabsf(m.omega_dps);
    if (v_abs > s_result.peak_v_mm_s) s_result.peak_v_mm_s = v_abs;
    if (w_abs > s_result.peak_omega_dps) s_result.peak_omega_dps = w_abs;

    if (vbat < SYSID_VBAT_MIN_V) vbat = SYSID_VBAT_MIN_V;
    bool sat = false;
    int16_t pwm_l = VoltageToPWM(v_left, vbat, &sat);
    int16_t pwm_r = VoltageToPWM(v_right, vbat, &sat);
    if (sat) {
        if (s_result.sat_ticks == 0u) s_result.sat_first_tick = s_result.ticks;
        s_result.sat_ticks++;
    }
    if (s_is_ff && s_ff.vd_sat) {
        if (s_result.vd_sat_ticks == 0u) s_result.vd_sat_first_tick = s_result.ticks;
        s_result.vd_sat_ticks++;
    }
    s_result.ticks++;

    Motor_Drive(pwm_l, pwm_r);

    if (s_is_ff) {
        s_dbg.step = (float)s_ff.cfg.step_no;
        s_dbg.seg = (float)FfTest_LogSeg(&s_ff);
        s_dbg.v_cmd = s_ff.v_cmd;
        s_dbg.v_ref = s_ff.v_ref;
        s_dbg.a_ref = s_ff.a_ref;
        s_dbg.v_s = s_ff.v_cmd;
        s_dbg.v_ff = s_ff.v_ff;
        s_dbg.v_p = s_ff.v_p;
        s_dbg.v_i = s_ff.v_i;
        s_dbg.v_d = s_ff.v_d;
        s_dbg.theta = s_ff.theta_rad;
        s_dbg.v_d0 = s_ff.v_d0;
        s_dbg.dv = s_ff.dv_out;
        s_dbg.v_d_meas = s_ff.v_d_meas;
    } else {
        s_dbg.step = (float)(s_step.cfg.step_base + s_step.step + 1u);
        s_dbg.seg = (float)s_step.seg;
        s_dbg.v_cmd = s_step.v_cmd;
    }
    s_dbg.pwm_l = (float)pwm_l;
    s_dbg.pwm_r = (float)pwm_r;
}
