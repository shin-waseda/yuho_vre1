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
    // 補正を受けない推定の位置と、補正の量(2026-11-05 に x_mm・vl_ref・vr_ref・f_ff と入れ替えた。列の上限のため)
    Logger_AddField("est_x", &d->est_x_mm);
    Logger_AddField("est_y", &d->est_y_mm);
    Logger_AddField("wall_ofs", &d->wall_ofs_deg);
    Logger_AddField("corr_along", &d->corr_along_mm);
    // 車輪
    Logger_AddField("vl", &d->vl);
    Logger_AddField("vr", &d->vr);
    Logger_AddField("pwm_l", &d->pwm_l);
    Logger_AddField("pwm_r", &d->pwm_r);
    Logger_AddField("ff_l", &d->ff_l);
    Logger_AddField("ff_r", &d->ff_r);
    Logger_AddField("i_s", &d->i_s);
    // 回転
    Logger_AddField("omega_ref", &d->target_omega_dps);
    Logger_AddField("gyro_z", &d->gyro_z_dps);
    Logger_AddField("angle_ref", &d->angle_ref_deg);
    Logger_AddField("angle", &d->angle_deg);
    Logger_AddField("fb_d", &d->fb_d);
    Logger_AddField("lat_err", &d->lat_err_mm); // これで 40 列(LOGGER_MAX_FIELDS)
    // 壁センサー・電池
    Logger_AddField("ad_l", &d->ad_l);
    Logger_AddField("ad_fl", &d->ad_fl);
    Logger_AddField("ad_fr", &d->ad_fr);
    Logger_AddField("ad_r", &d->ad_r);
    Logger_AddField("vbat", &d->vbat);
    // 6軸(ログに残すだけ。ジャイロ Z は gyro_z)
    Logger_AddField("acc_x", &d->acc_x);
    Logger_AddField("acc_y", &d->acc_y);
    Logger_AddField("acc_z", &d->acc_z);
    Logger_AddField("gyro_x", &d->gyro_x_dps);
    Logger_AddField("gyro_y", &d->gyro_y_dps);
    // イベント
    Logger_AddEventFields();

    Logger_SetDecimation(decimation);
}
