function L = sf_load(name)
% sf_load  同定のログ(CSV)を読み，SI 単位の量に直す
%
%   L = sf_load('e1t3_500mms_p050_0001')     % 拡張子なし．置き場所は sf_config
%
% 出す量: t [s]，step，seg，VL・VR [V](pwm/4199 × vbat)，Vs = (VL+VR)/2，Vd = (VR−VL)/2，
%         v [m/s](左右の車輪の平均)，w [rad/s](ジャイロ z)，vd0 [V](直進の区間で測った V_d．あれば)

cfg = sf_config();
f = fullfile(cfg.logDir, [name '.csv']);
T = readtable(f, 'VariableNamingRule', 'preserve');

L.name = name;
L.t    = T.time_s;
L.step = T.step;
L.seg  = T.seg;
L.VL   = T.pwm_l / cfg.pwmMax .* T.vbat;
L.VR   = T.pwm_r / cfg.pwmMax .* T.vbat;
L.Vs   = (L.VL + L.VR) / 2;
L.Vd   = (L.VR - L.VL) / 2;
L.v    = (T.vl + T.vr) / 2 / 1000;
L.w    = deg2rad(T.gyro_z);
if ismember('v_d0', T.Properties.VariableNames)
    % 直進の区間の最後に測った V_d の平均．ターンの区間にだけ値が入る(v_d_meas は E1C のターン中に不具合があるので使わない)
    i = find(T.v_d0 ~= 0, 1, 'last');
    if isempty(i), L.vd0 = NaN; else, L.vd0 = T.v_d0(i); end
else
    L.vd0 = NaN;
end
end
