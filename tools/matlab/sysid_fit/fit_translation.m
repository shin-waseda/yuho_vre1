function R = fit_translation()
% fit_translation  並進のモデル b dv/dt = V_s − a v − V0 を，電圧のステップ(E1S)に出力誤差法で当てはめる
%
%   R = fit_translation
%
% 使うログ: 床 e1s_200_1500mv_0025〜0032(6軸の軸を直した後)，浮かせた e1sl_200_1500mv_0001〜0002．
% 1 本のファイルに 0.1V 刻みの段(step)が並ぶ．区間1(seg 1)が電圧をかけている 1 s．
% 中止(seg 9)を含む段と，動き出しが不安定・直線から外れる低い電圧(< 0.45V)は使わない．
%
% やり方: かけた電圧 V_s(t) をそのままモデルに入れて速さをシミュレーションし，全部の段の
% 速さの二乗誤差の平均を fminsearch で最小にする(定常の値を外挿しないので，T_acc 1 s でも偏らない)．
% 止まっていて V_s ≤ V0 の間は動かない(クーロン摩擦)．

cfg = sf_config();
floorFiles = arrayfun(@(k) sprintf('e1s_200_1500mv_%04d', k), 25:32, 'UniformOutput', false);
liftFiles  = {'e1sl_200_1500mv_0001', 'e1sl_200_1500mv_0002'};

F = collect(floorFiles, 0.45);
Lf = collect(liftFiles, 0.45);      % 浮かせると 0.4V 以下で直線から外れる(ギアの摩擦．推測)
fprintf('床: %d 段，浮かせた: %d 段\n', numel(F.V0), numel(Lf.V0));

opt = optimset('MaxFunEvals', 4000, 'MaxIter', 4000, 'TolX', 1e-6, 'TolFun', 1e-10);
p0 = log([0.5 0.2 0.3]);              % a，V0，b の始めの値(丸めた値)
pf = exp(fminsearch(@(p) cost(exp(p), F, cfg.dt), p0, opt));
pl = exp(fminsearch(@(p) cost(exp(p), Lf, cfg.dt), p0, opt));

R.floor = struct('a', pf(1), 'V0', pf(2), 'b', pf(3), 'tau', pf(3)/pf(1), ...
    'rms', sqrt(cost(pf, F, cfg.dt)), 'rmsPrev', sqrt(cost([cfg.prev.a cfg.prev.V0 cfg.prev.b], F, cfg.dt)));
R.lift  = struct('a', pl(1), 'V0', pl(2), 'b', pl(3), 'tau', pl(3)/pl(1), 'rms', sqrt(cost(pl, Lf, cfg.dt)));
R.nFloor = numel(F.V0);  R.nLift = numel(Lf.V0);
R.frictionFloor = pf(2) - pl(2);

fprintf('床      : a = %.3f V/(m/s)，V0 = %.3f V，b = %.3f V/(m/s^2)，tau = %.3f s，RMS %.1f mm/s(前の値では %.1f mm/s)\n', ...
    R.floor.a, R.floor.V0, R.floor.b, R.floor.tau, 1000*R.floor.rms, 1000*R.floor.rmsPrev);
fprintf('浮かせた: a = %.3f V/(m/s)，V0 = %.3f V，b = %.3f V/(m/s^2)，tau = %.3f s，RMS %.1f mm/s\n', ...
    R.lift.a, R.lift.V0, R.lift.b, R.lift.tau, 1000*R.lift.rms);
fprintf('床の転がり抵抗(V0 の差) %.3f V\n', R.frictionFloor);
save(fullfile(cfg.outDir, 'translation.mat'), 'R');

%% 図: ステップ応答(浮かせた / 床)
f = figure('Color', 'w', 'Position', [100 100 1200 430], 'Visible', 'off');
tl = tiledlayout(f, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
drawSteps(nexttile(tl), Lf, pl, cfg, cfg.c.blue, ...
    sprintf('浮かせた：a = %.3f，V_0 = %.3f V，\\tau = %.3f s', pl(1), pl(2), pl(3)/pl(1)), [0.5 0.6 0.8 1.0 1.2]);
drawSteps(nexttile(tl), F, pf, cfg, cfg.c.orange, ...
    sprintf('床：a = %.3f，V_0 = %.3f V，\\tau = %.2f s', pf(1), pf(2), pf(3)/pf(1)), [0.5 0.6 0.7 0.8 0.9]);
exportgraphics(f, fullfile(cfg.figDir, 'fit_translation.png'), 'Resolution', 200);
close(f);
end

%% ---------------------------------------------------------------
function D = collect(files, vMin)
% 区間1 の段を集め，長さをそろえた行列(足りない所は NaN)にする
segs = {};
for i = 1:numel(files)
    L = sf_load(files{i});
    for s = unique(L.step(L.seg == 1))'
        in = L.step == s;
        if any(L.seg(in) == 9), continue; end          % 中止を含む段
        idx = find(in & L.seg == 1);
        if numel(idx) < 300, continue; end
        V = L.Vs(idx);
        if mean(V) < vMin, continue; end
        segs{end+1} = struct('V', V, 'v', L.v(idx), 'Vm', mean(V), 'file', files{i}); %#ok<AGROW>
    end
end
n = max(cellfun(@(s) numel(s.V), segs));
D.V = nan(n, numel(segs));  D.v = D.V;
for j = 1:numel(segs)
    m = numel(segs{j}.V);
    D.V(1:m, j) = segs{j}.V;  D.v(1:m, j) = segs{j}.v;
end
D.V0 = cellfun(@(s) s.Vm, segs);
D.v0 = D.v(1, :);
end

function vs = simulate(p, V, v0, dt)
a = p(1); V0 = p(2); b = p(3);
vs = nan(size(V));
v = v0;
for k = 1:size(V, 1)
    vs(k, :) = v;
    Vk = V(k, :);
    dv = (Vk - a*v - V0) / b;
    stuck = (v <= 0) & (Vk <= V0);
    dv(stuck) = 0;
    v = max(v + dt*dv, 0);
end
end

function J = cost(p, D, dt)
vs = simulate(p, D.V, max(D.v0, 0), dt);
e = vs - D.v;
J = mean(e(~isnan(e)).^2);
end

function drawSteps(ax, D, p, cfg, col, ttl, showV)
hold(ax, 'on'); box(ax, 'off'); grid(ax, 'on');
vs = simulate(p, D.V, max(D.v0, 0), cfg.dt);
t = (0:size(D.V, 1)-1)' * cfg.dt;
first = true;
for V = showV
    [d, j] = min(abs(D.V0 - V));
    if d > 0.03, continue; end
    ok = ~isnan(D.v(:, j));
    h1 = plot(ax, t(ok), D.v(ok, j), '-', 'Color', col, 'LineWidth', 1.6);
    h2 = plot(ax, t(ok), vs(ok, j), '--', 'Color', cfg.c.dark, 'LineWidth', 1.1);
    last = find(ok, 1, 'last');
    text(ax, t(last) + 0.01, vs(last, j), sprintf('%.1f V', D.V0(j)), 'FontSize', 9);
    if first, h1.DisplayName = '実測'; h2.DisplayName = 'モデル'; first = false;
    else, h1.HandleVisibility = 'off'; h2.HandleVisibility = 'off'; end
end
xlim(ax, [0 1.12]);
xlabel(ax, '電圧をかけてからの時間 [s]'); ylabel(ax, '速さ v [m/s]');
title(ax, ttl, 'FontWeight', 'normal', 'FontSize', 10);
legend(ax, 'Location', 'northwest', 'Box', 'off');
end
