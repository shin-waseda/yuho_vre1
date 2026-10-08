function D = study_data(files, wheel, sel)
%STUDY_DATA 同定の演習で使うデータを、ログから取り出す(演習1〜3で共通)。
%   D = study_data(files, 'l')            wheel は 'l'(左)，'r'(右)，'avg'(並進: 左右の平均の速さと電圧)
%   D = study_data(files, 'l', 'all')     sel = 'straight'(既定．前進の直線だけ)/ 'all'(下の除外だけ)
%   D.t, D.v[mm/s], D.a[mm/s²](実測の速さを微分), D.u[V](実際にかけた電圧), D.ok(同定に使う行), D.file(何本目のログか)
%
%   電圧 u = pwm / PWM_MAX × vbat。機体は「電圧 → duty」の換算をしてから PWM を出すので、その逆をしている。
%   使わない行: 超信地旋回(横にこする摩擦が別にある)、ほぼ止まっている所(摩擦の符号が決まらない)、
%   PWM が上限に張り付いた所、ログの最初と最後(微分が乱れる)。
%   'straight' ではさらに、前進の直線(目標の速さ > 0、目標の角速度 = 0)だけにする。
%     後退は尻当て(壁に押し付けて止まっている)が多く、電圧の割に速さが出ないのでモデルに合わない。
%     スラロームの間は左右の車輪が機体の回転でつながるので、車輪1つのモデルから外れる。
if nargin < 3 || isempty(sel), sel = 'straight'; end
PWM_MAX = 4199;
files = cellstr(files);
D = struct('t', [], 'v', [], 'a', [], 'u', [], 'ok', logical([]), 'file', [], 'dt', []);
for k = 1:numel(files)
    p = files{k};
    if ~isfile(p), p = fullfile(yc.root(), p); end
    L = yc.load_log(p);
    if strcmp(wheel, 'avg')
        % 並進: 機体の重さは左右の車輪で分け合うので、左右をまとめると「機体1つ」のモデルになる
        v = 0.5 * (L.vl + L.vr);
        u = 0.5 * (L.pwm_l + L.pwm_r) / PWM_MAX .* L.vbat;
        pwm = max(abs(L.pwm_l), abs(L.pwm_r));
    else
        v = L.(['v' wheel]);
        u = L.(['pwm_' wheel]) / PWM_MAX .* L.vbat;
        pwm = abs(L.(['pwm_' wheel]));
    end
    t = L.time_s;
    dt = median(diff(t));
    % 速さの微分は雑音が大きいので、15ms ほどの移動平均でならしてから微分する
    w = max(3, round(0.015 / dt));
    a = gradient(movmean(v, w), t);
    ok = abs(v) > 20 & pwm < 0.98 * PWM_MAX;
    if isfield(L, 'omega_ref') && isfield(L, 'target')
        ok = ok & ~(abs(L.target) < 1 & abs(L.omega_ref) > 1);  % 超信地旋回を除く
        if strcmp(sel, 'straight')
            ok = ok & L.target > 0 & abs(L.omega_ref) < 1;
        end
    end
    ok([1:w, end-w+1:end]) = false;
    D.t = [D.t; t]; D.v = [D.v; v]; D.a = [D.a; a]; D.u = [D.u; u];
    D.ok = [D.ok; ok]; D.file = [D.file; k * ones(size(t))]; D.dt(k) = dt;
end
end
