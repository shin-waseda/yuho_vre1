function [groups, default_cols, hidden] = plot_groups()
%PLOT_GROUPS 同じ段に描く列の組(上から順。log_viewer.py の GROUPS と同じ)
%   groups: {段の名前, {列, ...}; ...}
groups = {
    '速度 [mm/s]',        {'target', 'vl', 'vr', 'vl_ref', 'vr_ref'}
    '加速度 [mm/s²]',     {'target_acc'}
    '距離 [mm]',          {'pos_ref', 'dist', 'x_mm'}
    '位置の補正 [mm/s]',  {'pos_corr'}
    '角速度 [dps]',       {'omega_ref', 'gyro_z', 'ang_corr'}
    '角度 [deg]',         {'angle_ref', 'angle'}
    '壁の補正 [deg]',     {'wall_ofs'}
    'PWM',                {'pwm_l', 'pwm_r'}
    '電圧 [V]',           {'ff_l', 'ff_r', 'i_l', 'i_r'}
    '壁センサー',         {'ad_l', 'ad_fl', 'ad_fr', 'ad_r'}
    '電池 [V]',           {'vbat'}
    };
default_cols = {'target', 'vl', 'vr', 'angle_ref', 'angle', 'ad_l', 'ad_fl', 'ad_fr', 'ad_r'};
hidden = {'time_s', 'ev', 'ev_a', 'ev_b', 'ev_c', 'ev_d', 'ev_e', 'ev_text', 'event'};
end
