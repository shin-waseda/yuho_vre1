function R = fit_slip_k_log(csv_paths, varargin)
%FIT_SLIP_K_LOG 置いた所・止まった所の写真と SLIP のログから、1 本ごとの滑りの分を出して K(と C)を決める(2026-11-21)。
%   R = fit_slip_k_log('M:/.../photos/slip_l90/slip_camera.csv')
%   R = fit_slip_k_log({'.../slip_l90/slip_camera.csv', '.../slip_turn/slip_camera.csv'}, 'FitC', true)
%
%   1 本ごとに:
%   1. 写真(analyze_slip_camera の slip_camera.csv)から、止まった所を置いた所から見た位置(前 +、左 +)にする。
%   2. ログの最後の est_x・est_y(機体がエンコーダとジャイロで思っていた止まった所。CAMERA はスリップのモデルを
%      切って走るので滑りは入っていない)を引く。差 = 滑りの分 + 速さによらないずれ。
%   3. 差を、ログの最後の向き(ジャイロ)で、出る向き(前後 along)と外(曲がる中心の反対、横 lat)に分ける。
%   4. 滑りの見込みを、ログの速さ((vl + vr) / 2)とジャイロの角速度で β を進めて出す(機体の推定と同じ式
%      dβ/dt = (K v ω − β) / C。進む向き = 機体の向き − β)。β は K に比例するので、K = 1 の β を K 倍して使う。
%   5. 横の差 = 走りの種類ごとの定数 + 見込み(K, C) を最小二乗で当てはめる。定数は K, C ごとに平均で決める。
%      1 本ごとの K も出す(定数を引いた差 ÷ 見込み × K)。重み付き平均(見込みの 2 乗で重み)は当てはめの K とほぼ同じ。
%      ただの平均は、見込みの小さい遅い走りのばらつきで大きく振れるので使わない。
%
%   写真の当てはめの確かめ: 写真から出した向きの変わり方(止まった所 − 置いた所)とジャイロの向きの変わり方の差が
%   MaxDthDeg より大きい本は、LED の取り違えとして外す(2026-11-20 の LARGE90 では、取り違えは 11〜44°、
%   合っているものは 1.7° 以内だった)。
%
%   ログ: logs/slip/slip<kind>_<v>_NNNN.csv。CSV の (kind, v) ごとに、番号の大きい方から本数だけを古い順に使う
%   (同じ速さを 2 回走ったときは番号の小さい方が先)。1 行目の SLIP_RUN の kind・v と合うかを確かめる。
%   CSV に log の列(ファイル名)があれば、それを使う。
%
%   オプション
%     LogDir     ログのフォルダ(既定 <リポジトリ>/logs/slip)
%     C          固定する C[s](既定 0.02 = SLALOM_SLIP_C_S)
%     FitC       C も探す(既定 false)
%     Use        'lat'(既定)/ 'both'(前後も使う。前後は止まり方・車輪の滑りも入る)
%     MaxDthDeg  写真とジャイロの向きの変わり方の差の上限[deg](既定 2)
%     MinLeds    当てはめた LED の数の下限(置いた所・止まった所とも。既定 3)
%     Plot       図を出す(既定 true)。PlotFile を渡すとその PNG に書く
%     KCheck     確かめる K(既定 [])。渡すと、その K での残り RMS と、当てはめの幅に入るかを出す
%                (例: 大回り 90° で出した 0.0092 が、斜めの旋回でも合うか)
%     KModel     モデルありの走り(DIAG の kind 40〜47)で機体が使った K(既定 0.009 = SLALOM_SLIP_K)
%
%   斜めの旋回(DIAG。2026-11-22): 曲がり方が 1 回でないので、前後・横は最後の向きで分ける(135 は最後の向きが置いた向きと
%   同じなので、横は左 +)。モデルあり(kind 40〜47)の走りでは、ログの est_x・est_y に機体の見込んだ β が入っているので、
%   当てはめの K は「機体の K に足りない分」(0 なら補正が合っている)になる。K は負まで探し、KModel を足した値も出す。
%   モデルあり・なしは混ぜずに別々に当てはめる。
%
%   R: 1 本 1 行の table(使わなかった本は used = false と理由)。R.Properties.UserData に K, C, 幅, 定数。
p = inputParser;
p.addParameter('LogDir', '');
p.addParameter('C', 0.02);
p.addParameter('FitC', false);
p.addParameter('Use', 'lat');
p.addParameter('MaxDthDeg', 2);
p.addParameter('MinLeds', 3);
p.addParameter('Plot', true);
p.addParameter('PlotFile', '');
p.addParameter('KCheck', []);
p.addParameter('KModel', 0.009);
p.parse(varargin{:});
o = p.Results;
if isempty(o.LogDir)
    o.LogDir = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'logs', 'slip');
end
csv_paths = cellstr(csv_paths);

% ---- 写真の表をまとめる ----
T = table();
for i = 1:numel(csv_paths)
    t = readtable(csv_paths{i}, 'TextType', 'string');
    t.src = repmat(string(csv_paths{i}), height(t), 1);
    if ~ismember('log', t.Properties.VariableNames), t.log = strings(height(t), 1); end
    T = [T; t(:, {'run', 'name', 'kind', 'v', 'file', 'x_mm', 'y_mm', 'th_deg', 'n_leds_used', ...
                  'x0_mm', 'y0_mm', 'th0_deg', 'n0_leds_used', 'src', 'log'})]; %#ok<AGROW>
end
n = height(T);
model_on = T.kind >= 40 & T.kind <= 47; % DIAG のモデルあり
if any(model_on) && ~all(model_on)
    error('fit_slip_k_log:mixed', 'モデルあり(kind 40〜47)となしの走りが混ざっている。別々に当てはめる');
end
model_on = all(model_on);

% ---- ログを割り当てる ----
[g, gk, gv] = findgroups(T.kind, T.v);
for j = 1:max(g)
    rows = find(g == j & T.log == "");
    if isempty(rows), continue; end
    d = dir(fullfile(o.LogDir, sprintf('slip%d_%d_*.csv', gk(j), gv(j))));
    names = string({d.name});
    pat = sprintf('^slip%d_%d_\\d{4}\\.csv$', gk(j), gv(j)); % _dup1 などは使わない
    names = sort(names(arrayfun(@(s) ~isempty(regexp(s, pat, 'once')), names)));
    if numel(names) < numel(rows)
        warning('fit_slip_k_log:log', 'slip%d_%d のログが %d 本しかない(写真は %d 本)', gk(j), gv(j), numel(names), numel(rows));
    end
    use = names(max(1, end - numel(rows) + 1):end);
    for k = 1:min(numel(rows), numel(use))
        T.log(rows(k)) = use(k);
    end
end

% ---- 1 本ずつ ----
z = nan(n, 1);
R = table(T.run, T.name, T.kind, T.v, T.file, T.log, false(n, 1), strings(n, 1), z, z, z, z, z, z, z, ...
    'VariableNames', {'run', 'name', 'kind', 'v', 'file', 'log', 'used', 'why', 'dth_cam', 'dth_gyro', ...
                      'd_lat', 'd_along', 'ay_max_g', 'k_run', 'w'});
L = cell(n, 1);   % ログ(t, v, om, th)
for i = 1:n
    if T.log(i) == ""
        R.why(i) = "ログがない"; continue;
    end
    if any(isnan([T.x_mm(i) T.x0_mm(i)]))
        R.why(i) = "写真の位置が出なかった"; continue;
    end
    if T.n_leds_used(i) < o.MinLeds || T.n0_leds_used(i) < o.MinLeds
        R.why(i) = sprintf("LED が少ない(置いた所 %d、止まった所 %d)", T.n0_leds_used(i), T.n_leds_used(i)); continue;
    end
    lg = read_log(fullfile(o.LogDir, T.log(i)));
    if ~isnan(lg.kind) && (lg.kind ~= T.kind(i) || abs(lg.v_run - T.v(i)) > 0.5)
        R.why(i) = sprintf("ログの SLIP_RUN が kind %d v %g", lg.kind, lg.v_run); continue;
    end
    % 写真: 止まった所を置いた所から見た位置(前 +、左 +)と向きの変わり方
    a = deg2rad(T.th0_deg(i));
    dx = T.x_mm(i) - T.x0_mm(i); dy = T.y_mm(i) - T.y0_mm(i);
    cam = [dx * cos(a) + dy * sin(a), -dx * sin(a) + dy * cos(a)];
    R.dth_cam(i) = wrap180(T.th_deg(i) - T.th0_deg(i));
    R.dth_gyro(i) = lg.th_deg(end);
    if abs(wrap180(R.dth_cam(i) - R.dth_gyro(i))) > o.MaxDthDeg
        R.why(i) = sprintf("向きの変わり方が写真 %.1f° とジャイロ %.1f° で違う(LED の取り違え)", R.dth_cam(i), R.dth_gyro(i));
        continue;
    end
    [u, nv] = exit_frame(lg.th_deg(end));
    dd = cam - [lg.est_x(end) lg.est_y(end)];
    R.d_along(i) = dd * u';
    R.d_lat(i) = dd * nv';
    R.ay_max_g(i) = max(abs(lg.v .* deg2rad(lg.om_dps))) * 1e-3 / 9.80665;
    L{i} = lg;
    R.used(i) = true;
end
for i = find(~R.used)'
    fprintf('#%d %s v %g (%s): 使わない — %s\n', R.run(i), R.name(i), R.v(i), R.file(i), R.why(i));
end
iu = find(R.used);
if numel(iu) < 3, error('fit_slip_k_log:few', '使える本が %d 本しかない', numel(iu)); end
fprintf('%d 本を使う(%s)\n', numel(iu), strjoin(compose("%s %d", unique(R.name(iu)), ...
    arrayfun(@(s) sum(R.name(iu) == s), unique(R.name(iu)))), '、'));

% ---- 当てはめ ----
Ks = 0:0.0002:0.03;
if model_on, Ks = -0.015:0.0002:0.015; end % 機体の K に足りない分(負なら機体の K が大きすぎる)
ik0 = find(abs(Ks) < 1e-9, 1); % K = 0
if o.FitC, Cs = [0 0.005 0.01 0.02 0.03 0.05 0.08 0.12 0.2]; else, Cs = o.C; end
P = cell(numel(Cs), 1);   % P{ic}: 本 × K × [along lat] の見込み
for ic = 1:numel(Cs)
    P{ic} = zeros(numel(iu), numel(Ks), 2);
    for m = 1:numel(iu)
        P{ic}(m, :, :) = slip_pred(L{iu(m)}, Ks, Cs(ic));
    end
end
grp = findgroups(R.name(iu));
use_along = strcmpi(o.Use, 'both');
sse = nan(numel(Cs), numel(Ks));
for ic = 1:numel(Cs)
    for ik = 1:numel(Ks)
        rl = R.d_lat(iu) - P{ic}(:, ik, 2);
        ml = splitapply(@mean, rl, grp);
        s = sum((rl - ml(grp)) .^ 2);
        if use_along
            ra = R.d_along(iu) - P{ic}(:, ik, 1);
            ma = splitapply(@mean, ra, grp);
            s = s + sum((ra - ma(grp)) .^ 2);
        end
        sse(ic, ik) = s;
    end
end
[smin, idx] = min(sse(:));
[icb, ikb] = ind2sub(size(sse), idx);
K = Ks(ikb); C = Cs(icb);
nobs = numel(iu) * (1 + use_along);
npar = 1 + o.FitC + max(grp) * (1 + use_along);
s2 = smin / max(1, nobs - npar);
kr = Ks(sse(icb, :) <= smin + s2);

pl = P{icb}(:, ikb, 2); pa = P{icb}(:, ikb, 1);
b_lat = splitapply(@mean, R.d_lat(iu) - pl, grp);
b_along = splitapply(@mean, R.d_along(iu) - pa, grp);
gname = splitapply(@(s) s(1), R.name(iu), grp);
% 1 本ごとの K(定数を引いた横の差 ÷ 見込み × K)。見込みが小さい本は振れるので、見込みの 2 乗で重みを付ける
R.k_run(iu) = K * (R.d_lat(iu) - b_lat(grp)) ./ pl;
R.w(iu) = pl .^ 2;
kw = sum(R.k_run(iu) .* R.w(iu)) / sum(R.w(iu));

fprintf('\nK = %.4f(幅 %.4f〜%.4f)  C = %.3f s%s  使ったずれ %s  残り RMS %.2f mm\n', K, min(kr), max(kr), C, ...
    ternary(o.FitC, '', '(固定)'), o.Use, sqrt(s2));
fprintf('K = 0(滑りなし)の残りの二乗和 %.1f → %.1f\n', sse(icb, ik0), smin);
if model_on
    fprintf('モデルありの走り: 上の K は機体の K(%.4f)に足りない分。足すと %.4f(幅 %.4f〜%.4f)。K = 0 が幅に入れば補正は合っている\n', ...
        o.KModel, o.KModel + K, o.KModel + min(kr), o.KModel + max(kr));
end
if ~isempty(o.KCheck)
    [~, ikc] = min(abs(Ks - o.KCheck));
    fprintf('K = %.4f の確かめ: 残り RMS %.2f mm(一番よい K で %.2f)、当てはめの幅に%s\n', Ks(ikc), ...
        sqrt(sse(icb, ikc) / max(1, nobs - npar)), sqrt(s2), ternary(sse(icb, ikc) <= smin + s2, '入る', '入らない'));
end
fprintf('1 本ごとの K: 重み付き平均 %.4f、中央値 %.4f、ただの平均 %.4f(遅い走りで振れる)\n', kw, ...
    median(R.k_run(iu)), mean(R.k_run(iu)));
if o.FitC
    fprintf('C ごとの一番よい K と残りの二乗和:\n');
    for ic = 1:numel(Cs)
        [s, j] = min(sse(ic, :));
        fprintf('  C %.3f: K %.4f  SSE %.1f\n', Cs(ic), Ks(j), s);
    end
end
fprintf('走りの種類ごとの定数(速さによらないずれ。写真 − ログ):\n');
for j = 1:numel(gname)
    sel = iu(grp == j);
    fprintf('  %-6s 横(外 +) %+6.1f mm  前後 %+6.1f mm  写真とジャイロの向きの変わり方の差 %+.2f°(%d 本)\n', ...
        gname(j), b_lat(j), b_along(j), mean(wrap180(R.dth_cam(sel) - R.dth_gyro(sel))), numel(sel));
end
fprintf('\n 本  走り    速さ  横加速度  横の差  見込み  残り   前後の差  見込み  1 本の K\n');
for m = 1:numel(iu)
    i = iu(m);
    fprintf('%3d  %-6s %5g  %5.2fG  %6.2f  %6.2f  %+5.2f   %7.2f  %6.2f   %.4f\n', R.run(i), R.name(i), R.v(i), ...
        R.ay_max_g(i), R.d_lat(i), pl(m), R.d_lat(i) - pl(m) - b_lat(grp(m)), R.d_along(i), pa(m), R.k_run(i));
end

R.Properties.UserData = struct('K', K, 'K_range', [min(kr) max(kr)], 'C', C, 'rms_mm', sqrt(s2), ...
    'K_weighted', kw, 'groups', gname, 'b_lat', b_lat, 'b_along', b_along);

if o.Plot
    f = figure('Color', 'w', 'Position', [100 100 1100 420]);
    tl = tiledlayout(f, 1, 2);
    ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
    cols = lines(numel(gname)); % 左右の図で同じ色
    for j = 1:numel(gname)
        sel = find(grp == j);
        [~, o2] = sort(R.ay_max_g(iu(sel)));
        plot(ax, R.ay_max_g(iu(sel)), R.d_lat(iu(sel)) - b_lat(j), 'o', 'Color', cols(j, :), 'DisplayName', gname(j));
        plot(ax, R.ay_max_g(iu(sel(o2))), pl(sel(o2)), '-', 'Color', cols(j, :), 'HandleVisibility', 'off');
    end
    xlabel(ax, '横加速度の最大 [G]'); ylabel(ax, '横の差 − 走りの定数 [mm](外 +)');
    legend(ax, 'Location', 'northwest');
    title(ax, sprintf('点は写真 − ログ、線は見込み(K %.4f、C %.3f)', K, C));
    ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
    for j = 1:numel(gname)
        sel = iu(grp == j);
        scatter(ax, R.ay_max_g(sel), R.k_run(sel), 20 + 200 * R.w(sel) / max(R.w(iu)), cols(j, :), 'filled', ...
            'DisplayName', gname(j));
    end
    yline(ax, K, '-', sprintf('当てはめ %.4f', K));
    xlabel(ax, '横加速度の最大 [G]'); ylabel(ax, '1 本ごとの K'); title(ax, '1 本ごとの K(丸の大きさは重み)');
    if ~isempty(o.PlotFile)
        exportgraphics(f, o.PlotFile, 'Resolution', 120);
        fprintf('wrote %s\n', o.PlotFile);
    end
end
end

function lg = read_log(path)
% get_log.py / sd_import.py の CSV。列の名前で読む(ev_text はカンマを含むことがあるので、数の列だけ読む)
fid = fopen(path, 'r', 'n', 'UTF-8');
c = onCleanup(@() fclose(fid));
head = strsplit(strtrim(fgetl(fid)), ',');
nnum = find(strcmp(head, 'ev_text')) - 1;
if isempty(nnum), nnum = numel(head); end
D = textscan(fid, [repmat('%f', 1, nnum) '%[^\n]'], 'Delimiter', ',', 'ReturnOnError', false);
col = @(nm) D{strcmp(head, nm)};
lg.t = col('time_s');
lg.v = 0.5 * (col('vl') + col('vr'));
lg.om_dps = col('gyro_z');
lg.th_deg = col('angle');
lg.est_x = col('est_x');
lg.est_y = col('est_y');
% 走りの種類と速さ(1 行目の SLIP_RUN の ev_a・ev_b)。lg.v は速さの列(車輪の平均)
lg.kind = NaN; lg.v_run = NaN;
k = find(startsWith(string(D{end}), "SLIP_RUN"), 1);
if ~isempty(k)
    ea = col('ev_a'); eb = col('ev_b');
    lg.kind = ea(k); lg.v_run = eb(k);
end
end

function Pk = slip_pred(lg, Ks, C)
% K ごとの滑りの見込み [along lat](K × 1 の β。機体の推定と同じ式)
dt = [0; diff(lg.t)];
v = lg.v; om = deg2rad(lg.om_dps); th = deg2rad(lg.th_deg);
target = v * 1e-3 .* om;           % K = 1 の β の目標
b = zeros(size(v));
for i = 2:numel(v)
    if C <= 0
        b(i) = target(i);
    else
        b(i) = b(i - 1) + (1 - exp(-dt(i) / C)) * (target(i) - b(i - 1));
    end
end
[u, nv] = exit_frame(lg.th_deg(end));
Pk = zeros(numel(Ks), 2);
for k = 1:numel(Ks)
    bk = Ks(k) * b;
    dx = sum(v .* dt .* (cos(th - bk) - cos(th)));
    dy = sum(v .* dt .* (sin(th - bk) - sin(th)));
    Pk(k, :) = [[dx dy] * u', [dx dy] * nv'];
end
end

function [u, nv] = exit_frame(th_end_deg)
% 出る向き u と外 nv(置いた所の座標: 前 +x、左 +y)。右に曲がった(向きが減った)なら外は出る向きの左、左なら右。
% 向きがほとんど変わらない走り(DIAG の 135。右 135° → 左 135°)は、ジャイロのずれで符号が変わらないよう左を + にする
a = deg2rad(th_end_deg);
u = [cos(a) sin(a)];
s = -sign(th_end_deg);
if abs(th_end_deg) < 22.5, s = 1; end
nv = s * [-sin(a) cos(a)];
end

function a = wrap180(a)
a = mod(a + 180, 360) - 180;
end

function r = ternary(c, a, b)
if c, r = a; else, r = b; end
end
