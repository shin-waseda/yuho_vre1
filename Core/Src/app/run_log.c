#include "app/run_log.h"

#include "app/control_loop.h"

void RunLog_Setup(const char *dir, const char *file, uint32_t decimation) {
    const ControlDebug *d = App_GetControlDebug();

    Logger_Init();
    Logger_SetDirName(dir);
    Logger_SetFileName(file);

    // 並進
    Logger_AddField("target", &d->target_mm_s);
    Logger_AddField("target_acc", &d->target_acc);
    Logger_AddField("pos_ref", &d->pos_ref);
    Logger_AddField("dist", &d->dist_mm);
    Logger_AddField("pos_corr", &d->pos_corr);
    Logger_AddField("x_mm", &d->x_mm);
    // 車輪
    Logger_AddField("vl", &d->vl);
    Logger_AddField("vr", &d->vr);
    Logger_AddField("vl_ref", &d->vl_ref);
    Logger_AddField("vr_ref", &d->vr_ref);
    Logger_AddField("pwm_l", &d->pwm_l);
    Logger_AddField("pwm_r", &d->pwm_r);
    Logger_AddField("ff_l", &d->ff_l);
    Logger_AddField("ff_r", &d->ff_r);
    Logger_AddField("i_l", &d->i_l);
    Logger_AddField("i_r", &d->i_r);
    // 回転
    Logger_AddField("omega_ref", &d->target_omega_dps);
    Logger_AddField("gyro_z", &d->gyro_z_dps);
    Logger_AddField("angle_ref", &d->angle_ref_deg);
    Logger_AddField("angle", &d->angle_deg);
    Logger_AddField("ang_corr", &d->ang_corr_dps);
    Logger_AddField("wall_ofs", &d->wall_offset_deg);
    // 壁センサー・電池
    Logger_AddField("ad_l", &d->ad_l);
    Logger_AddField("ad_fl", &d->ad_fl);
    Logger_AddField("ad_fr", &d->ad_fr);
    Logger_AddField("ad_r", &d->ad_r);
    Logger_AddField("vbat", &d->vbat);
    // イベント
    Logger_AddEventFields();

    Logger_SetDecimation(decimation);
}
