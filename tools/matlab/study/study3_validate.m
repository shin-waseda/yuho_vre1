function fitp = study3_validate(th, files)
%STUDY3_VALIDATE 演習3: 求めたモデルを、同定に使っていないログで確かめる(検証)。
%   実際にかけた電圧 u をモデルに入れて速さを計算し(シミュレーション)、実際の速さと重ねる。
%   「1歩先の予測」(演習2)はいつも実測から出発するので甘く見える。シミュレーションは自分の出した値だけで
%   進むので、モデルの誤差が積み重なって見える。モデルの良さはこちらで判断する。
%   前進の直線の区間(0.5 秒以上続く所)ごとに、区間の始めの実際の速さから計算を始める。
%
%   study3_validate              % 演習1のモデルと params.h の FF を、確かめ用のログ(study_files('check'))で比べる
%   th: 演習1の戻り値(th.l, th.r, th.avg = [FRIC; GAIN; ACC])
if nargin < 1 || isempty(th)
    th = study1_ff_regression();
    close(gcf);
end
if nargin < 2 || isempty(files)
    files = study_files('check');
end
p = yc.read_params();
names = {'l', 'r', 'avg'};
titles = {'左の車輪', '右の車輪', '並進(左右の平均)'};
mnames = {'ログから求めたモデル', 'params.h の FF のモデル'};
figure('Name', '演習3: シミュレーションで確かめる', 'Color', 'w');
tl = tiledlayout(3, 1, 'TileSpacing', 'compact');
fitp = struct();
for w = 1:3
    s = names{w};
    S = titles{w};
    D = study_data(files, s);
    if strcmp(s, 'avg')
        now = 0.5 * (ff_of(p, 'L') + ff_of(p, 'R'));
    else
        now = ff_of(p, upper(s));
    end
    models = {th.(s), now};
    runs = find_runs(D, 0.5);
    vs = nan(numel(D.t), 2);
    for r = 1:size(runs, 1)
        j = runs(r, 1):runs(r, 2);
        for k = 1:2
            vs(j, k) = simulate(models{k}, D.t(j), D.u(j), D.v(j(1)));
        end
    end
    in = ~isnan(vs(:, 1));
    ax = nexttile(tl);
    j = find(D.file == 1);
    vplot = D.v(j); vplot(~in(j)) = nan;
    plot(ax, D.t(j), vplot, 'Color', [0.5 0.5 0.5], 'LineWidth', 1.4); hold(ax, 'on');
    labels = {'実際の速さ'};
    for k = 1:2
        e = D.v(in) - vs(in, k);
        f = 100 * (1 - norm(e) / norm(D.v(in) - mean(D.v(in))));   % 一致度[%](100 で完全に一致)
        fitp.(s)(k) = f;
        fprintf('[%s] %s: 一致度 %.1f%%，誤差 RMS %.1f mm/s，平均の誤差 %+.1f mm/s (区間 %d 個)\n', ...
            S, mnames{k}, f, sqrt(mean(e.^2)), mean(e), size(runs, 1));
        plot(ax, D.t(j), vs(j, k), 'LineWidth', 1.0);
        labels{end+1} = sprintf('%s (一致度 %.0f%%)', mnames{k}, f); %#ok<AGROW>
    end
    legend(ax, labels, 'Location', 'best');
    title(ax, sprintf('%s: 実際の電圧を入れて計算した速さ [mm/s](前進の直線の区間だけ)', S));
    grid(ax, 'on');
end
xlabel(ax, 'time [s]');
end

function ff = ff_of(p, S)
% params.h の FF の係数 [FRIC; GAIN; ACC](S は 'L' か 'R')
ff = [p.(['VELOCITY_FF_FRIC_' S]); p.(['VELOCITY_FF_GAIN_' S]); p.(['VELOCITY_FF_ACC_' S])];
end

function runs = find_runs(D, min_s)
% 同じログの中で ok が続く区間 [始め, 終わり] のうち、min_s 秒以上のもの
ok = D.ok(:)';
brk = [true, diff(D.file(:)') ~= 0];
d = diff([0, ok & true, 0]);
st = find(d == 1); en = find(d == -1) - 1;
runs = zeros(0, 2);
for k = 1:numel(st)
    % ログの境目をまたぐ区間は分ける
    cut = find(brk(st(k) + 1:en(k))) + st(k);
    edges = [st(k), cut; cut - 1, en(k)]';
    for e = 1:size(edges, 1)
        if D.t(edges(e, 2)) - D.t(edges(e, 1)) >= min_s
            runs(end+1, :) = edges(e, :); %#ok<AGROW>
        end
    end
end
end

function v = simulate(theta, t, u, v0)
% ACC·dv/dt + GAIN·v + FRIC·sign(v) = u  を、ログの間隔で前へ進める(オイラー法)。
% これは  T·dv/dt + v = K·(u − FRIC·sign(v))  (K = 1/GAIN，T = ACC/GAIN)と同じ式。
F = theta(1); G = theta(2); A = theta(3);
v = zeros(size(t));
v(1) = v0;
for k = 2:numel(t)
    h = t(k) - t(k - 1);
    dv = (u(k - 1) - F * sign(v(k - 1)) - G * v(k - 1)) / A;   % 加速度 = (電圧 − 摩擦 − 速さの分) / ACC
    v(k) = v(k - 1) + h * dv;
end
end
