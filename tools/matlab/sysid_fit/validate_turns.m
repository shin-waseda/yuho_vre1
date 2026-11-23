function V = validate_turns(Dmode)
% validate_turns  当てはめに使っていない「大会のターンを旋回の FF だけで曲がった」ログ(E1C)でモデルを確かめる
%
%   V = validate_turns            % out/rotation_run.mat(D は本ごとの V_d0)の値で
%   V = validate_turns('const')   % out/rotation_const.mat(D は共通)の値で
%
% E1C: 区間3 がターン(旋回は FF だけ．並進は PI あり)．実測の V_d と v をモデルに入れて ω を計算し，
% ターンの一定の区間(目標の ω が一定の所．ここでは区間3 の中央の 50%)の平均の比(実測 / モデル)と一致度を出す．

cfg = sf_config();
if nargin < 1, Dmode = 'run'; end
s = load(fullfile(cfg.outDir, ['rotation_' Dmode '.mat']));  P = s.R.P;
Pprev = struct('bw', cfg.prev.bw, 'sigma', cfg.prev.sigma, 'Ap', cfg.prev.Ap, 'Am', cfg.prev.Am, ...
    'k0v', cfg.prev.k0v, 'k0', cfg.prev.k0, 'D', cfg.prev.D);   % 機体で使っている共通の D

conds = {'300mms_s90', '小回り 0.3 m/s'; '500mms_s90', '小回り 0.5 m/s'; '600mms_s90', '小回り 0.6 m/s';
         '700mms_s90', '小回り 0.7 m/s'; '600mms_l90', '大回り 90° 0.6 m/s'; '500mms_l180', '大回り 180° 0.5 m/s'};
% 走る前から様子がおかしかった本(yuho-sysid の開発日記 2026-11-03 の E1C)．
% 0002: 直進の区間で V_d が FB の上限に張り付き，ターンの前に 7.4° 回っていた．0001: ターンが 0.43 倍しか回らなかった(原因は不明)．
excluded = {'e1c_300mms_s90_R_0001', 'e1c_300mms_s90_R_0002'};
rows = {};
for c = 1:size(conds, 1)
    for side = 'LR'
        S = rot_collect(sprintf('e1c_%s_%s_*', conds{c, 1}, side), 3);
        for j = 1:numel(S.D)
            if any(strcmp(S.name{j}, excluded)), continue; end
            n = S.len(j);
            W  = rot_sim(P,     S.Vd(1:n, j), S.v(1:n, j), Dof(P, S, j), S.w0(j), cfg.dt);
            Wp = rot_sim(Pprev, S.Vd(1:n, j), S.v(1:n, j), Dof(Pprev, S, j), S.w0(j), cfg.dt);
            y = S.w(2:n, j);  m = W(1:n-1);  mp = Wp(1:n-1);
            mid = round(0.25*(n-1)) : round(0.75*(n-1));
            rows(end+1, :) = {conds{c, 2}, side, S.name{j}, ...
                mean(y(mid)) / mean(m(mid)), mean(y(mid)) / mean(mp(mid)), ...
                fitp(y, m), fitp(y, mp)}; %#ok<AGROW>
        end
    end
end
V = cell2table(rows, 'VariableNames', {'cond', 'side', 'file', 'ratio', 'ratioPrev', 'fit', 'fitPrev'});
G = groupsummary(V, {'cond', 'side'}, 'mean', {'ratio', 'ratioPrev', 'fit', 'fitPrev'});
disp(G(:, {'cond', 'side', 'GroupCount', 'mean_ratio', 'mean_ratioPrev', 'mean_fit', 'mean_fitPrev'}));
fprintf('全体: 一定の区間の比 MATLAB %.2f±%.2f / 前 %.2f±%.2f，一致度の中央値 MATLAB %.1f %% / 前 %.1f %%\n', ...
    mean(V.ratio), std(V.ratio), mean(V.ratioPrev), std(V.ratioPrev), median(V.fit), median(V.fitPrev));
save(fullfile(cfg.outDir, ['validate_turns_' Dmode '.mat']), 'V', 'G');

%% 図: 3 つの条件で，左右 1 本ずつ
show = {'500mms_s90', '小回り 0.5 m/s'; '700mms_s90', '小回り 0.7 m/s'; '600mms_l90', '大回り 90° 0.6 m/s'};
f = figure('Color', 'w', 'Position', [100 100 1300 380], 'Visible', 'off');
tl = tiledlayout(f, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for c = 1:size(show, 1)
    ax = nexttile(tl); hold(ax, 'on'); box(ax, 'off'); grid(ax, 'on');
    for side = 'LR'
        S = rot_collect(sprintf('e1c_%s_%s_*', show{c, 1}, side), 3);
        [~, j] = max(S.len);                         % いちばん長い(中止していない)本
        n = S.len(j);  t = (0:n-1) * cfg.dt * 1000;
        W = rot_sim(P, S.Vd(1:n, j), S.v(1:n, j), Dof(P, S, j), S.w0(j), cfg.dt);
        sg = S.dir(j);
        if side == 'L', col = cfg.c.blue; nm = '左回り'; else, col = cfg.c.orange; nm = '右回り'; end
        plot(ax, t, sg*rad2deg(S.w(1:n, j)), '-', 'Color', col, 'LineWidth', 1.4, 'DisplayName', [nm ' 実測']);
        plot(ax, t, sg*rad2deg(W), '--', 'Color', col*0.7, 'LineWidth', 1.4, 'DisplayName', [nm ' モデル']);
    end
    title(ax, show{c, 2}, 'FontWeight', 'normal', 'FontSize', 10);
    xlabel(ax, 'ターン開始からの時間 [ms]');
    if c == 1, ylabel(ax, '角速度(回る向きを正) [deg/s]'); legend(ax, 'Location', 'south', 'Box', 'off', 'FontSize', 8); end
end
exportgraphics(f, fullfile(cfg.figDir, ['validate_turns_' Dmode '.png']), 'Resolution', 200);
close(f);
end

function p = fitp(y, m)
p = 100 * (1 - norm(y - m) / norm(y - mean(y)));
end

function D = Dof(P, S, j)
if isfield(P, 'D') && ~isnan(P.D), D = P.D * ones(1, numel(j)); else, D = S.D(j); end
end
