function th = study1_ff_regression(files, sel)
%STUDY1_FF_REGRESSION 演習1: ログから車輪のモデル(FF の係数)を最小二乗法で求める。
%   モデル:  u = FRIC·sign(v) + GAIN·v + ACC·a      (u: 電圧[V], v: 速さ[mm/s], a: 加速度[mm/s²])
%   これは「未知数 θ = [FRIC; GAIN; ACC] について1次式」なので、行列 X = [sign(v) v a] を作れば
%   u ≈ X·θ を最小にする θ は  θ = X \ u  (MATLAB の左割り算 = 最小二乗)で1行で求まる。
%
%   study1_ff_regression                     % 既定: 今の駆動系で走った探索のログの、前進の直線だけ
%   study1_ff_regression([], 'all')          % 後退・スラロームも入れると、どう崩れるか
%   左(l)・右(r)の車輪ごとのほかに、並進(avg: 左右の平均の速さと電圧)のモデルも求める。
%   最後に「左の電圧は右の加速にも効くか」(左右のつながり)も調べる。
if nargin < 1 || isempty(files)
    files = study_files();
end
if nargin < 2 || isempty(sel), sel = 'straight'; end
p = yc.read_params();
names = {'l', 'r', 'avg'};
titles = {'左の車輪', '右の車輪', '並進(左右の平均)'};
th = struct();
figure('Name', '演習1: FF の係数を最小二乗で', 'Color', 'w');
tl = tiledlayout(2, 3, 'TileSpacing', 'compact');
for w = 1:3
    s = names{w};
    D = study_data(files, s, sel);
    i = D.ok;
    X = [sign(D.v(i)), D.v(i), D.a(i)];
    y = D.u(i);
    theta = X \ y;                       % ← 最小二乗法はこの1行
    yhat = X * theta;
    r2 = 1 - sum((y - yhat).^2) / sum((y - mean(y)).^2);   % 決定係数(1 に近いほどよく説明できている)
    now = params_ff(p, s);
    fprintf('\n[%s] 使った行 %d / %d\n', titles{w}, nnz(i), numel(i));
    fprintf('          %10s %12s %12s\n', 'FRIC[V]', 'GAIN', 'ACC');
    fprintf('  ログから %10.3f %12.6f %12.7f\n', theta);
    fprintf('  params.h %10.3f %12.6f %12.7f\n', now);
    fprintf('  → K = 1/GAIN = %.0f mm/s/V，T = ACC/GAIN = %.3f s，R² = %.3f，残差 RMS %.3f V\n', ...
        1 / theta(2), theta(3) / theta(2), r2, sqrt(mean((y - yhat).^2)));
    th.(s) = theta;

    % 図: 実際の電圧とモデルの電圧(最初のログの一部)
    ax = nexttile(tl, w);
    j = find(D.file == 1);
    j = j(1:min(numel(j), round(8 / D.dt(1))));   % 8 秒分
    Xj = [sign(D.v(j)), D.v(j), D.a(j)];
    uj = D.u(j); uj(~D.ok(j)) = nan;              % 使わなかった行は描かない
    plot(ax, D.t(j), uj, 'Color', [0.6 0.6 0.6]); hold(ax, 'on');
    plot(ax, D.t(j), Xj * theta, 'LineWidth', 1.0);
    plot(ax, D.t(j), Xj * now(:), '--', 'LineWidth', 0.8);
    legend(ax, '実際の電圧', 'ログから求めたモデル', 'params.h の FF', 'Location', 'best');
    title(ax, sprintf('%s: 電圧 [V]', titles{w})); xlabel(ax, 'time [s]'); grid(ax, 'on');

    % 図: 各項の寄与(電圧を「摩擦・速さ・加速」に分ける)
    ax = nexttile(tl, w + 3);
    plot(ax, D.t(j), Xj .* theta', 'LineWidth', 1.0);
    legend(ax, '摩擦', '速さ', '加速', 'Location', 'best');
    title(ax, sprintf('%s: モデルの電圧の内訳', titles{w})); xlabel(ax, 'time [s]'); grid(ax, 'on');
end

% ---- 左右のつながり: 左の電圧 = FRIC + GAIN·v_L + A1·a_L + A2·a_R ----
% 車輪ごとのモデルが正しければ A2 = 0 のはず。機体の重さを左右で分け合っていれば A1 と A2 は同じくらいになる。
DL = study_data(files, 'l', sel);
DR = study_data(files, 'r', sel);
i = DL.ok & DR.ok;
X = [sign(DL.v(i)), DL.v(i), DL.a(i), DR.a(i)];
y = DL.u(i);
c = X \ y;
r2 = 1 - sum((y - X * c).^2) / sum((y - mean(y)).^2);
fprintf('\n[左右のつながり] 左の電圧 = %.3f + %.6f·v_L + %.7f·a_L + %.7f·a_R   (R² = %.3f)\n', c, r2);
fprintf('  a_R の係数が a_L と同じくらいなら、左の車輪は「自分の加速」だけでなく「機体の加速」に電圧を使っている。\n');
th.coupled_l = c;
end

function now = params_ff(p, s)
% params.h の FF の係数 [FRIC GAIN ACC](並進は左右の平均)
if strcmp(s, 'avg')
    now = 0.5 * (params_ff(p, 'l') + params_ff(p, 'r'));
else
    S = upper(s);
    now = [p.(['VELOCITY_FF_FRIC_' S]), p.(['VELOCITY_FF_GAIN_' S]), p.(['VELOCITY_FF_ACC_' S])];
end
end
