function make_blog_figs(outDir)
% make_blog_figs  はてなブログ記事「yuho の並進・旋回モデル」の図を作る
%
%   make_blog_figs            % このファイルと同じフォルダの ../images に PNG を書く
%   make_blog_figs(outDir)    % 書き出し先を指定
%
% 入力：同じフォルダの blog_fig_data.json（ログから取り出した，図に要る分だけのデータ）
% 図 7（h(kappa)）と図 9（FF と FF+FB のシミュレーション）はモデルの式から計算する．
% 図 8（大会のターンを FF だけで）はこのデータに入っていないので作らない．

here = fileparts(mfilename('fullpath'));
if nargin < 1, outDir = fullfile(here, '..', 'images'); end
if ~exist(outDir, 'dir'), mkdir(outDir); end
D = jsondecode(fileread(fullfile(here, 'blog_fig_data.json')));

C.blue   = [0.165 0.435 0.859];
C.orange = [0.851 0.510 0.169];
C.grey   = [0.60 0.60 0.60];
C.dark   = [0.27 0.27 0.27];
C.teal   = [0.075 0.537 0.478];

%% 図 1・2：ステップ応答（浮かせた / 床）
stepFig(D.lift,  0.5576, 0.0771, 0.082, C.blue,   'a = 0.558,  V_0 = 0.077 V,  \tau = 0.082 s', fullfile(outDir, 'fig01_lift_step.png'), C);
stepFig(D.floor, 0.561,  0.210,  0.509, C.orange, 'a = 0.561,  V_0 = 0.210 V,  \tau = 0.51 s',  fullfile(outDir, 'fig02_floor_step.png'), C);

%% 図 3：定常の速さと電圧
f = newFig(750, 440); hold on; box off; grid on;
ss = D.ss; cl = [ss.clean];
plot([ss(~cl).vf], [ss(~cl).V], 'o', 'Color', C.grey, 'MarkerSize', 7, 'LineWidth', 1.3, 'DisplayName', '床（タイヤにごみ）');
plot([ss(cl).vf],  [ss(cl).V],  'o', 'Color', C.orange, 'MarkerFaceColor', C.orange, 'MarkerSize', 6, 'DisplayName', '床（タイヤを掃除）');
plot([D.ssl.vf], [D.ssl.V], 'o', 'Color', C.blue, 'MarkerFaceColor', C.blue, 'MarkerSize', 6, 'DisplayName', '浮かせた');
x = [0 1.6]; plot(x, 0.561*x + 0.210, '-',  'Color', C.orange, 'LineWidth', 1.8, 'DisplayName', '床のモデル  V = 0.561 v + 0.210');
x = [0 2.6]; plot(x, 0.5576*x + 0.0771, '--', 'Color', C.blue, 'LineWidth', 1.8, 'DisplayName', '浮かせたモデル  V = 0.558 v + 0.077');
plot([0.02 0.02], [0.0771 0.210], '-', 'Color', C.dark, 'LineWidth', 1, 'HandleVisibility', 'off');
text(0.06, 0.04, '差 0.133 V ＝ 床の転がり抵抗', 'FontSize', 9, 'Color', C.dark);
xlim([0 2.7]); ylim([0 1.6]);
xlabel('落ち着いたときの速さ v_\infty [m/s]'); ylabel('かけた電圧 V [V]');
legend('Location', 'northwest', 'Box', 'off', 'FontSize', 9);
saveFig(f, fullfile(outDir, 'fig03_steady_state.png'));

%% 図 4：FF だけ，b = 0.286
r = D.ff_b0286;
f = newFig(800, 380); hold on; box off; grid on;
plot(r.t, r.vr_, '--', 'Color', C.dark, 'LineWidth', 1.3, 'DisplayName', '目標 v_{ref}');
plot(r.t, r.v, '-', 'Color', C.orange, 'LineWidth', 1.8, 'DisplayName', '実測 v');
segLines(r.seg, max(r.vr_)*1.12);
ylim([0 max(r.vr_)*1.15]); xlim([0 r.seg(3) + 0.03]);
xlabel('時間 [s]'); ylabel('速さ [m/s]');
title('FF だけ，b = 0.286（ステップから決めた値）', 'FontWeight', 'normal', 'FontSize', 11);
legend('Location', 'southeast', 'Box', 'off');
saveFig(f, fullfile(outDir, 'fig04_ff_b0286.png'));

%% 図 5：FF だけ と FF+PI
a = D.ff_only; b = D.ff_pi;
f = newFig(800, 560);
subplot(2, 1, 1); hold on; box off; grid on;
plot(a.t, a.vr_, '--', 'Color', C.dark, 'LineWidth', 1.3, 'DisplayName', '目標 v_{ref}');
plot(a.t, a.v, '-', 'Color', C.orange, 'LineWidth', 1.8, 'DisplayName', 'FF だけ（b = 0.32）');
plot(b.t, b.v, '-', 'Color', C.blue, 'LineWidth', 1.8, 'DisplayName', 'FF＋PI');
segLines(a.seg, max(a.vr_)*1.14);
ylim([0 max(a.vr_)*1.18]); xlim([0 max(a.t(end), b.t(end))]);
ylabel('速さ [m/s]');
legend('Location', 'south', 'Orientation', 'vertical', 'Box', 'off', 'FontSize', 9);
subplot(2, 1, 2); hold on; box off; grid on;
yline0(C);
plot(a.t, a.err, '-', 'Color', C.orange, 'LineWidth', 1.6, 'DisplayName', 'FF だけ');
plot(b.t, b.err, '-', 'Color', C.blue, 'LineWidth', 1.6, 'DisplayName', 'FF＋PI');
xlim([0 max(a.t(end), b.t(end))]);
xlabel('時間 [s]'); ylabel('v - v_{ref} [mm/s]');
legend('Location', 'southwest', 'Orientation', 'horizontal', 'Box', 'off', 'FontSize', 9);
saveFig(f, fullfile(outDir, 'fig05_ff_vs_pi.png'));

%% 図 6：一定電圧のブレーキ（1.0 m/s から）
bs = D.brake; [~, ix] = sort(-[bs.Vb]); bs = bs(ix);
f = newFig(750, 420); hold on; box off; grid on;
n = numel(bs);
for i = 1:n
    al = 0.35 + 0.65*(i-1)/max(n-1, 1);
    t = bs(i).t(:); m = t <= 0.6;
    col = C.blue*al + [1 1 1]*(1-al);
    h1 = plot(t(m), bs(i).v(m), '-', 'Color', col, 'LineWidth', 1.8);
    mv = bs(i).model(m); tm = t(m); k = mv >= 0;
    h2 = plot(tm(k), mv(k), '--', 'Color', C.dark, 'LineWidth', 1.1);
    tt = tm(k); vv = mv(k);
    if bs(i).Vb == 0, lab = '0 V'; else, lab = sprintf('%.1f V', bs(i).Vb); end
    if i == n   % 一番強いブレーキは右に置くと隣の線に重なるので，線の左下に置く
        text(tt(end) - 0.04, 0.03, lab, 'FontSize', 9, 'Color', [0.2 0.2 0.2], 'HorizontalAlignment', 'right');
    else
        text(max(tt(end), t(find(m, 1, 'last'))) + 0.006, vv(end) + 0.02, lab, 'FontSize', 9, 'Color', [0.2 0.2 0.2]);
    end
    if i == n
        set(h1, 'DisplayName', '実測'); set(h2, 'DisplayName', 'モデル（加速と同じ式）');
    else
        set(h1, 'HandleVisibility', 'off'); set(h2, 'HandleVisibility', 'off');
    end
end
xlim([0 0.6]); ylim([0 1.12]);
xlabel('ブレーキを始めてからの時間 [s]'); ylabel('速さ [m/s]');
legend('Location', 'northeast', 'Box', 'off');
saveFig(f, fullfile(outDir, 'fig06_brake.png'));

%% 図 7：こすれる抵抗の行き先 h(kappa)
VK = [0.3 0.5 0.75 1.0 1.5]; K0 = [1.03 1.14 0.80 0.89 0.92];
f = newFig(950, 400);
AA = [0.32 0.216]; nm = {'左回り  A_+ = 0.32 V', '右回り  A_- = 0.216 V'};
for p = 1:2
    subplot(1, 2, p); hold on; box off; grid on;
    patch([6.7 15.7 15.7 6.7], [0 0 1.5 1.5], [0.93 0.93 0.93], 'EdgeColor', 'none');
    text(11.2, 0.12, {'大会のターン', 'で使う範囲'}, 'HorizontalAlignment', 'center', 'FontSize', 9, 'Color', [0.45 0.45 0.45]);
    k = linspace(0, 30, 300);
    for i = 1:numel(VK)
        al = 0.35 + 0.65*(i-1)/(numel(VK)-1);
        plot(k, AA(p)*asinh(k/K0(i)), '-', 'Color', C.teal*al + [1 1 1]*(1-al), 'LineWidth', 1.8);
    end
    text(30.3, AA(p)*asinh(30/K0(4)), '速さ 0.3〜1.5 m/s', 'FontSize', 8.5, 'Color', [0.35 0.35 0.35]);
    s0 = AA(p)/K0(4);
    plot([0 1.4/s0], [0 1.4], '--', 'Color', C.dark, 'LineWidth', 1.1);
    text(1.4/s0 + 0.3, 1.3, {'原点の傾きのまま', '伸ばした直線'}, 'FontSize', 8.5, 'Color', C.dark);
    xlim([0 36]); ylim([0 1.5]);
    title(nm{p}, 'FontWeight', 'normal', 'FontSize', 11);
    xlabel('曲がり具合 |\kappa| [1/m]');
    if p == 1, ylabel('こすれる抵抗の行き先 |h| [V]'); end
end
saveFig(f, fullfile(outDir, 'fig07_h_kappa.png'));

%% 図 9：モデルで計算した 90° ターン（FF だけ / FF+FB，D が 0.05 V ずれている）
c = struct('v', 0.5, 'th', 90, 'w', 450, 'a', 8000);
A = simTurn(c, 0.05, 0.32, false, VK, K0);
B = simTurn(c, 0.05, 0.32, true,  VK, K0);
f = newFig(800, 560);
subplot(2, 1, 1); hold on; box off; grid on;
plot(A.t, A.wr, '--', 'Color', C.dark, 'LineWidth', 1.3, 'DisplayName', '目標 \omega_{ref}');
plot(A.t, A.w, '-', 'Color', C.orange, 'LineWidth', 1.8, 'DisplayName', 'FF だけ');
plot(B.t, B.w, '-', 'Color', C.blue, 'LineWidth', 1.8, 'DisplayName', 'FF＋FB');
ylim([0 520]); xlim([0 A.t(end)]); ylabel('角速度 [deg/s]');
title('モデルで計算した 90° ターン（0.5 m/s，450 deg/s，8000 deg/s^2）．機体の D が 0.05 V ずれている場合', 'FontWeight', 'normal', 'FontSize', 10);
legend('Location', 'northeast', 'Box', 'off', 'FontSize', 9);
subplot(2, 1, 2); hold on; box off; grid on;
yline0(C);
plot(A.t, A.e, '-', 'Color', C.orange, 'LineWidth', 1.6, 'DisplayName', 'FF だけ');
plot(B.t, B.e, '-', 'Color', C.blue, 'LineWidth', 1.6, 'DisplayName', 'FF＋FB');
xlim([0 A.t(end)]); xlabel('ターン開始からの時間 [ms]'); ylabel('角度のずれ [°]');
legend('Location', 'southwest', 'Box', 'off', 'FontSize', 9);
saveFig(f, fullfile(outDir, 'fig09_sim_ff_fb.png'));
fprintf('図 9：ターン後の角度のずれ  FF だけ %.2f°，FF+FB %.2f°\n', A.e(end), B.e(end));
fprintf('書き出し先：%s\n', outDir);
end

% ---------------------------------------------------------------------------
function stepFig(sets, a, V0, tau, col, ttl, file, C)
f = newFig(800, 420); hold on; box off; grid on;
t = (0:99)'*0.01 + 0.005; tm = linspace(0, 1, 200);
n = numel(sets);
for i = 1:n
    al = 0.35 + 0.65*(i-1)/max(n-1, 1);
    h1 = plot(t, sets(i).v, '-', 'Color', col*al + [1 1 1]*(1-al), 'LineWidth', 1.8);
    h2 = plot(tm, max(sets(i).V - V0, 0)/a*(1 - exp(-tm/tau)), '--', 'Color', C.dark, 'LineWidth', 1.1);
    text(1.01, sets(i).v(end), sprintf('%.1f V', sets(i).V), 'FontSize', 9, 'Color', [0.35 0.35 0.35]);
    if i == n
        set(h1, 'DisplayName', '実測'); set(h2, 'DisplayName', 'モデル');
    else
        set(h1, 'HandleVisibility', 'off'); set(h2, 'HandleVisibility', 'off');
    end
end
xlim([0 1.0]); yl = ylim; ylim([0 yl(2)]);
xlabel('電圧を入れてからの時間 [s]'); ylabel('速さ v [m/s]');
title(ttl, 'FontWeight', 'normal', 'FontSize', 10);
legend('Location', 'southeast', 'Orientation', 'horizontal', 'Box', 'off');
saveFig(f, file);
end

function segLines(seg, top)
names = {'加速', '一定', '減速'};
for i = 1:3
    plot([seg(i) seg(i)], [0 top*1.02], ':', 'Color', [0.73 0.73 0.73], 'LineWidth', 0.8, 'HandleVisibility', 'off');
    text(seg(i) + 0.01, top, names{i}, 'FontSize', 9, 'Color', [0.47 0.47 0.47], 'VerticalAlignment', 'top');
end
end

function yline0(C)
xl = [-1e3 1e4];
plot(xl, [0 0], '-', 'Color', C.grey, 'LineWidth', 0.8, 'HandleVisibility', 'off');
end

function f = newFig(w, h)
f = figure('Color', 'w', 'Units', 'pixels', 'Position', [100 100 w h]);
set(f, 'DefaultAxesFontSize', 10, 'DefaultAxesXColor', [0.35 0.35 0.35], 'DefaultAxesYColor', [0.35 0.35 0.35]);
end

function saveFig(f, file)
set(f, 'PaperPositionMode', 'auto');
print(f, file, '-dpng', '-r200');
close(f);
end

function out = simTurn(c, dD, Atrue, fb, VK, K0)
% 1 ms ごとのシミュレーション．FF は A+ = 0.32，A- = 0.216 を使い，機体の D は FF より dD だけずれている．
B = 9.2e-3; SIG = 3.02e-3; DN = -0.128;
th = deg2rad(c.th); wm = deg2rad(c.w); al = deg2rad(c.a);
ta = wm/al; tc = (th - wm*ta)/wm; T = 2*ta + tc; dt = 1e-3; n = round((T + 0.1)/dt) + 1;
cc = min(c.v*dt/SIG, 1);
om = 0; F = 0; Fff = 0; thp = 0; thr = 0; thm = 0;
Vq = [DN DN]; meas = [0 0];
out.t = zeros(n, 1); out.wr = out.t; out.w = out.t; out.e = out.t;
for kq = 1:n
    t  = (kq-1)*dt; wr = trap(t, al, T, wm);
    tn = kq*dt;     wn = trap(tn, al, T, wm);
    ar = (wn - wr)/dt;
    thr = thr + wr*dt;
    Fff = Fff + cc*(hfun(wr, c.v, 0.32, 0.216, VK, K0) - Fff);
    Vff = B*ar + DN + Fff;
    wmeas = meas(1); thm = thm + wmeas*dt;
    u = 0;
    if fb, u = max(-1.5, min(1.5, 0.386*(wr - wmeas) + 8.28*(thr - thm))); end
    Vq(end+1) = Vff + u; Vapp = Vq(1); Vq(1) = [];          %#ok<AGROW>
    om = om + dt/B*(Vapp - (DN + dD) - F);
    F  = F + cc*(hfun(om, c.v, Atrue, 0.216, VK, K0) - F);
    thp = thp + om*dt;
    meas(end+1) = om; meas(1) = [];                          %#ok<AGROW>
    out.t(kq) = t*1000; out.wr(kq) = rad2deg(wr); out.w(kq) = rad2deg(om); out.e(kq) = rad2deg(thp - thr);
end
end

function w = trap(t, al, T, wm)
w = min([al*t, al*(T - t), wm]);
if w < 0 || t > T, w = 0; end
end

function y = hfun(om, v, Ap, Am, VK, K0)
k = om/v;
if k >= 0, A = Ap; else, A = Am; end
y = A*asinh(k/interp1(VK, K0, min(max(v, VK(1)), VK(end))));
end
