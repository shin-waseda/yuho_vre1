function res = fit_slip(logs, slope, use_bias, refine)
%FIT_SLIP スリップアングルの K と C を SLALOM テストのログに合わせる(turn_sim.py の「合わせる」と同じ)。
%   res = yc.fit_slip(logs, slope, use_bias, refine)
%   logs: 同じ旋回の種類の yc.TurnLog の配列(または cell)
%   slope: 横の壁センサーの傾き [AD/mm]
%   use_bias: 右・左で共通の横のずれ b も合わせる(既定 true。右・左の両方のログが要る)
%   refine: 格子で探した後、fminsearch で細かく詰める(既定 true。false なら Python 版と同じ結果)
%
%   測った横のずれ(外が +) = スリップの分 + b × s。b は機体が右へずれて見える分で、
%   外は左に曲がると右・右に曲がると左なので s = +1(左に曲がった)/ −1(右)。
%   測った横のずれ = 壁センサーで見た横のずれ − 車輪とジャイロの軌道の横のずれ。
if nargin < 3 || isempty(use_bias), use_bias = true; end
if nargin < 4 || isempty(refine), refine = true; end
if iscell(logs), logs = [logs{:}]; end
FIT_K = linspace(0.0, 0.03, 151);
FIT_C = [0.0, linspace(0.002, 0.2, 100)];

data = struct('lg', {}, 'me', {}, 'ok', {}, 's', {});
for lg = logs(:)'
    meas = lg.sensor_lateral(lg.win, slope) - lg.kin_lateral(lg.win);
    ok = ~isnan(meas);
    if nnz(ok) >= 5
        data(end+1) = struct('lg', lg, 'me', meas(ok)', 'ok', ok, 's', 1.0 - 2.0 * lg.right); %#ok<AGROW>
    end
end
if isempty(data)
    error('fit_slip:nodata', '曲がった後に横の壁が見えている所がない');
end
n = sum(arrayfun(@(d) nnz(d.ok), data));

% ---- 格子で探す(Python 版と同じ) ----
best = [inf, 0, 0, 0];  % err, K, C, b
for C = FIT_C
    [err, b] = errs(FIT_K, C);
    [m, i] = min(err);
    if m < best(1)
        best = [m, FIT_K(i), C, b(i)];
    end
end
res.grid = struct('K', best(2), 'C', best(3), 'b', best(4), 'rms', sqrt(best(1) / n));

% ---- fminsearch で詰める(C は |C| として 0 以上にする) ----
res.refined = false;
if refine
    opt = optimset('TolX', 1e-7, 'TolFun', 1e-9, 'MaxFunEvals', 2000, 'Display', 'off');
    q0 = [best(2), max(best(3), 1e-3)];
    [q, fval] = fminsearch(@(q) errs(q(1), abs(q(2))), q0, opt);
    if fval < best(1)
        [~, b] = errs(q(1), abs(q(2)));
        best = [fval, q(1), abs(q(2)), b];
        res.refined = true;
    end
end
res.K = best(2); res.C = best(3); res.b = best(4);
res.rms = sqrt(best(1) / n);
res.n = n;
res.nlogs = numel(data);
res.names = arrayfun(@(d) string(d.lg.name), data);
res.use_bias = use_bias;
res.slope = slope;

% 比べる: スリップなし(b だけ)
b0 = 0.0;
if use_bias
    b0 = sum(arrayfun(@(d) d.s * sum(d.me), data)) / n;
end
res.rms0 = sqrt(sum(arrayfun(@(d) sum((b0 * d.s - d.me).^2), data)) / n);

notes = strings(0, 1);
if res.grid.K >= FIT_K(end) - 1e-9, notes(end+1) = "K が格子の端(格子は 0〜" + FIT_K(end) + ")"; end
if res.grid.C >= FIT_C(end) - 1e-9, notes(end+1) = "C が格子の端(格子は 0〜" + FIT_C(end) + ")"; end
if res.K < 0, notes(end+1) = "K が負(外へずれるはずの向きと逆)"; end
if use_bias && numel(unique([data.s])) < 2
    notes(end+1) = "片方の向きのログだけなので b と K を分けられない";
end
res.notes = notes;

txt = sprintf('ログに合わせた結果 (%d 本, %d 点):\n  K = %.4f, C = %.3f', res.nlogs, n, res.K, res.C);
if use_bias
    txt = [txt, sprintf(', b = %+.1f mm (右へ)', res.b)];
end
txt = [txt, sprintf('\n  残りのずれ RMS %.2f mm (スリップなしなら %.2f mm)', res.rms, res.rms0)];
if res.refined
    txt = [txt, sprintf('\n  (格子では K = %.4f, C = %.3f, RMS %.2f mm)', res.grid.K, res.grid.C, res.grid.rms)];
end
for k = 1:numel(notes)
    txt = [txt, sprintf('\n  ※ %s', notes(k))]; %#ok<AGROW>
end
txt = [txt, sprintf('\n  (傾き %g AD/mm を使った。傾きが違えば K も変わる)', slope)];
res.text = txt;

    function [err, b] = errs(Ks, C)
        % 各 K の二乗誤差の和と、そのときの最もよい b(閉じた式)
        nK = numel(Ks);
        models = cell(1, numel(data));
        b = zeros(nK, 1);
        for j = 1:numel(data)
            mo = data(j).lg.slip_model(Ks, C);
            models{j} = mo(:, data(j).ok);
            if use_bias
                b = b + data(j).s * sum(data(j).me - models{j}, 2);
            end
        end
        b = b / n;
        err = zeros(nK, 1);
        for j = 1:numel(data)
            r = models{j} + b * data(j).s - data(j).me;
            err = err + sum(r .* r, 2);
        end
    end
end
