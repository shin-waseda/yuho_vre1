function R = fit_rotation(Dmode)
% fit_rotation  旋回のモデル(rot_sim)の b_w，σ，A±，κ0(v) を，電圧の差のステップに出力誤差法で当てはめる
%
%   R = fit_rotation            % D は本ごとに直進の区間で測った V_d0
%   R = fit_rotation('const')   % D もすべての本に共通の未知のパラメータにする
%
% 使うログ: E1T3(0.5・1.0・1.5 m/s，区間3 で V_d = V_d0 ± ΔV を FB なしでかける．60° か 270° まで)と，
%           E1T の 0.3 m/s(e1t_300mms_*)．どちらも区間3 だけを使う．D は本ごとに直進の区間で測った V_d0．
% やり方: 実測の V_d と v をモデルに入れて ω をシミュレーションし，本ごとの誤差を
%         その本の ω の大きさ(RMS)で割った値の二乗平均を fminsearch で最小にする(小さく回る本も同じ重みにする)．
%         ジャイロの遅れ(約 1 ms)の分，実測は 1 サンプル後ろと比べる．

if nargin < 1, Dmode = 'run'; end
cfg = sf_config();
S = [rot_collect('e1t3_*', 3), rot_collect('e1t_300mms_*', 3)];
S = merge(S);
fprintf('本数 %d(速さ 0.3: %d，0.5: %d，1.0: %d，1.5: %d)\n', numel(S.D), ...
    sum(S.vNom == 0.3), sum(S.vNom == 0.5), sum(S.vNom == 1.0), sum(S.vNom == 1.5));

k0v = [0.3 0.5 1.0 1.5];
unpack = @(p) struct('bw', exp(p(1)), 'sigma', exp(p(2)), 'Ap', exp(p(3)), 'Am', exp(p(4)), ...
    'k0v', k0v, 'k0', exp(p(5:8)), 'D', dval(p));
p0 = log([0.01 5e-3 0.3 0.3 1 1 1 1]);        % 丸めた始めの値
if strcmp(Dmode, 'const'), p0(9) = -0.1; end
opt = optimset('MaxFunEvals', 3000, 'MaxIter', 3000, 'TolX', 1e-5, 'TolFun', 1e-8, 'Display', 'off');
J = @(p) cost(unpack(p), S, cfg.dt);
p = p0;
for rep = 1:3                                  % fminsearch を始め直して詰める
    [p, fv] = fminsearch(J, p, opt);
    fprintf('  %d 回目: 誤差 %.4f\n', rep, fv);
end
P = unpack(p);
Pprev = struct('bw', cfg.prev.bw, 'sigma', cfg.prev.sigma, 'Ap', cfg.prev.Ap, 'Am', cfg.prev.Am, ...
    'k0v', cfg.prev.k0v, 'k0', cfg.prev.k0, 'D', cfg.prev.D);   % 機体で使っている共通の D

R.P = P;  R.Dmode = Dmode;  R.Pprev = Pprev;  R.S = S;
[R.fit, R.nrmse] = fitPct(P, S, cfg.dt);
[R.fitPrev, R.nrmsePrev] = fitPct(Pprev, S, cfg.dt);
fprintf('b_w = %.2fe-3 V/(rad/s^2)，sigma = %.2f mm，A+ = %.3f V，A- = %.3f V\n', 1e3*P.bw, 1e3*P.sigma, P.Ap, P.Am);
if ~isnan(P.D), fprintf('D = %.3f V(すべての本に共通)\n', P.D); end
fprintf('kappa0(v) = %s /m (v = %s m/s)\n', mat2str(P.k0, 3), mat2str(k0v));
fprintf('一致度(本ごとの中央値): MATLAB の値 %.1f %%，前の値 %.1f %%\n', median(R.fit), median(R.fitPrev));
save(fullfile(cfg.outDir, ['rotation_' Dmode '.mat']), 'R');

%% 図: 代表の 3 本
pick = {'e1t3_500mms_p050_0001', 'e1t3_1000mms_m040_0001', 'e1t3_500mms_p080_a270_0001'};
ttl  = {'0.5 m/s，\DeltaV = +0.5 V', '1.0 m/s，\DeltaV = −0.4 V', '0.5 m/s，\DeltaV = +0.8 V'};
f = figure('Color', 'w', 'Position', [100 100 1300 380], 'Visible', 'off');
tl = tiledlayout(f, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:numel(pick)
    j = find(strcmp(S.name, pick{i}), 1);
    ax = nexttile(tl); hold(ax, 'on'); box(ax, 'off'); grid(ax, 'on');
    n = S.len(j);  t = (0:n-1) * cfg.dt * 1000;
    sel = @(X) X(1:n, j);
    W  = rot_sim(P, sel(S.Vd), sel(S.v), Dof(P, S, j), S.w0(j), cfg.dt);
    Wp = rot_sim(Pprev, sel(S.Vd), sel(S.v), Dof(Pprev, S, j), S.w0(j), cfg.dt);
    sg = S.dir(j);
    plot(ax, t, sg*rad2deg(sel(S.w)), '-', 'Color', cfg.c.orange, 'LineWidth', 1.4, 'DisplayName', '実測');
    plot(ax, t, sg*rad2deg(W), '-', 'Color', cfg.c.blue, 'LineWidth', 1.6, 'DisplayName', 'モデル(MATLAB で当てはめ)');
    plot(ax, t, sg*rad2deg(Wp), '--', 'Color', cfg.c.dark, 'LineWidth', 1.1, 'DisplayName', 'モデル(前の値)');
    title(ax, ttl{i}, 'FontWeight', 'normal', 'FontSize', 10);
    xlabel(ax, '電圧の差を入れてからの時間 [ms]');
    if i == 1, ylabel(ax, '角速度(回る向きを正) [deg/s]'); legend(ax, 'Location', 'southeast', 'Box', 'off'); end
end
exportgraphics(f, fullfile(cfg.figDir, ['fit_rotation_' Dmode '.png']), 'Resolution', 200);
close(f);
end

%% ---------------------------------------------------------------
function J = cost(P, S, dt)
W = rot_sim(P, S.Vd, S.v, Dof(P, S, 1:numel(S.D)), S.w0, dt);
E = W(1:end-1, :) - S.w(2:end, :);
e = sqrt(mean(E.^2, 1, 'omitnan')) ./ S.scale;
J = mean(e.^2);
if ~isfinite(J), J = 1e6; end
end

function [fit, nrmse] = fitPct(P, S, dt)
% 一致度 = 100(1 − ‖ω − ω_model‖ / ‖ω − mean(ω)‖)(本ごと)
W = rot_sim(P, S.Vd, S.v, Dof(P, S, 1:numel(S.D)), S.w0, dt);
M = numel(S.D);  fit = zeros(1, M);  nrmse = fit;
for j = 1:M
    n = S.len(j);
    y = S.w(2:n, j);  ym = W(1:n-1, j);
    fit(j) = 100 * (1 - norm(y - ym) / norm(y - mean(y)));
    nrmse(j) = sqrt(mean((y - ym).^2)) / S.scale(j);
end
end

function S = merge(A)
% rot_collect の結果を 1 つにまとめる
N = max(arrayfun(@(s) size(s.Vd, 1), A));
pad = @(X) [X; nan(N - size(X, 1), size(X, 2))];
S.Vd = []; S.v = []; S.w = [];
for s = A
    S.Vd = [S.Vd, pad(s.Vd)];  S.v = [S.v, pad(s.v)];  S.w = [S.w, pad(s.w)];
end
for fn = {'D', 'w0', 'name', 'vNom', 'dir', 'len'}
    S.(fn{1}) = [A.(fn{1})];
end
S.scale = sqrt(mean(S.w.^2, 1, 'omitnan'));
end

function D = Dof(P, S, j)
% D: P.D があればすべての本で共通，なければ本ごとの V_d0
if isfield(P, 'D') && ~isnan(P.D), D = P.D * ones(1, numel(j)); else, D = S.D(j); end
end

function D = dval(p)
% 9 番目のパラメータがあれば共通の D
if numel(p) >= 9, D = p(9); else, D = NaN; end
end
