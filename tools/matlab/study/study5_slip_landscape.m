function study5_slip_landscape(files, slope)
%STUDY5_SLIP_LANDSCAPE 演習5: スリップの K と C が「ログから決まるか」を、誤差の地形で見る(同定可能性)。
%   yc.fit_slip は誤差が一番小さい K, C を1組返すだけだが、誤差が K, C でどう変わるかを描くと、
%   「谷が細い向きの値はよく決まり、谷が平らな向きの値は決まらない」ことが目で見える。
%
%   study5_slip_landscape                % logs/slalom の SLALOM 試験のログ(5本)
if nargin < 1 || isempty(files)
    d = dir(fullfile(yc.root(), 'logs', 'slalom', 'turn_*.*'));
    files = {};
    for k = 1:numel(d)
        [~, st, ext] = fileparts(d(k).name);
        if strcmpi(ext, '.csv') && isfile(fullfile(d(k).folder, [st '.bin'])), continue; end
        files{end+1} = fullfile(d(k).folder, d(k).name); %#ok<AGROW>
    end
end
if nargin < 2 || isempty(slope), slope = 8.0; end
p = yc.read_params();
logs = cellfun(@(f) yc.TurnLog(f, p), files);

Ks = linspace(0, 0.04, 81);
Cs = linspace(0, 0.2, 41);
R = zeros(numel(Cs), numel(Ks));
data = struct('lg', {}, 'me', {}, 'ok', {}, 's', {});
for lg = logs(:)'
    meas = lg.sensor_lateral(lg.win, slope) - lg.kin_lateral(lg.win);
    ok = ~isnan(meas);
    if nnz(ok) >= 5
        data(end+1) = struct('lg', lg, 'me', meas(ok)', 'ok', ok, 's', 1 - 2 * lg.right); %#ok<AGROW>
    end
end
n = sum(arrayfun(@(d) nnz(d.ok), data));
for i = 1:numel(Cs)
    mo = cell(1, numel(data));
    b = zeros(numel(Ks), 1);
    for j = 1:numel(data)
        m = data(j).lg.slip_model(Ks, Cs(i));
        mo{j} = m(:, data(j).ok);
        b = b + data(j).s * sum(data(j).me - mo{j}, 2);
    end
    b = b / n;
    err = zeros(numel(Ks), 1);
    for j = 1:numel(data)
        r = mo{j} + b * data(j).s - data(j).me;
        err = err + sum(r .* r, 2);
    end
    R(i, :) = sqrt(err / n)';
end
[best, ib] = min(R(:));
[ic, ik] = ind2sub(size(R), ib);
fprintf('一番合うのは K = %.4f, C = %.3f (RMS %.2f mm)\n', Ks(ik), Cs(ic), best);
fprintf('RMS が一番よい値 + 0.1mm 以内に入る範囲: K %.4f〜%.4f, C %.3f〜%.3f\n', ...
    range_of(Ks, any(R <= best + 0.1, 1)), range_of(Cs, any(R <= best + 0.1, 2)'));

figure('Name', '演習5: K と C の誤差の地形', 'Color', 'w');
contourf(Ks, Cs, R, 30, 'LineStyle', 'none'); hold on
contour(Ks, Cs, R, best + [0.1 0.1], 'w', 'LineWidth', 1.5);
plot(Ks(ik), Cs(ic), 'r+', 'MarkerSize', 12, 'LineWidth', 2);
colorbar; xlabel('K [s²/m]'); ylabel('C [s]');
title(sprintf('曲がった後の横のずれの RMS [mm](白線: 最良 + 0.1mm．傾き %g AD/mm)', slope));
end

function r = range_of(x, mask)
r = [min(x(mask)), max(x(mask))];
end
