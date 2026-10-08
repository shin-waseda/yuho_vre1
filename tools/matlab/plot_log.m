function varargout = plot_log(files, cols)
%PLOT_LOG ログ(get_log.py が作った .csv、SD の .bin)を時系列のグラフにする(get_log.py --plot-file の代わり)。
%   同じ種類の列(速度・角度・壁センサーなど)は同じ段に描き、段の横軸(時刻)はそろえて動く。
%   ファイルを複数渡すと重ねて描く(凡例は「ファイル名: 列」)。
%
%   plot_log                                            % ファイルを選ぶ画面が出る(複数選べる)
%   plot_log('logs/vel_pid/step_20261003_120000.csv')
%   plot_log({'logs/slalom/turn_0001.bin', 'logs/slalom/turn_0002.bin'})
%   plot_log('logs/search/search_0008.bin', {'target', 'vl', 'vr', 'vbat'})   % 描く列を決める
%
%   cols を省くと全部の列(イベントの列などは除く)。組にない列は「その他」の段にまとめる。
if nargin < 1 || isempty(files)
    start = fullfile(yc.root(), 'logs');
    [f, d] = uigetfile({'*.csv;*.bin', 'log (*.csv, *.bin)'; '*.*', 'all'}, 'ログ', [start filesep], 'MultiSelect', 'on');
    if isequal(f, 0)
        varargout = cell(1, nargout);
        return
    end
    files = fullfile(d, cellstr(f));
end
files = cellstr(files);
for k = 1:numel(files)
    if ~isfile(files{k}) && isfile(fullfile(yc.root(), files{k}))
        files{k} = fullfile(yc.root(), files{k});  % リポジトリの一番上から見たパスでもよい
    end
end
[groups, ~, hidden] = yc.plot_groups();

logs = cell(size(files));
labels = cell(size(files));
all_cols = {};
for k = 1:numel(files)
    [logs{k}, names] = yc.load_log(files{k});
    [~, labels{k}] = fileparts(files{k});
    all_cols = [all_cols, setdiff(names, [hidden, all_cols], 'stable')]; %#ok<AGROW>
end
if nargin >= 2 && ~isempty(cols)
    all_cols = cellstr(cols);
end

% 段を決める
panels = cell(0, 2);
used = {};
for g = 1:size(groups, 1)
    sel = groups{g, 2}(ismember(groups{g, 2}, all_cols));
    if ~isempty(sel)
        panels(end+1, :) = {groups{g, 1}, sel}; %#ok<AGROW>
        used = [used, sel]; %#ok<AGROW>
    end
end
others = setdiff(all_cols, used, 'stable');
if ~isempty(others)
    panels(end+1, :) = {'その他', others};
end
if isempty(panels)
    error('plot_log:nocols', '描く列がない');
end

np = size(panels, 1);
fig = figure('Name', strjoin(labels, ', '), 'NumberTitle', 'off', 'Color', 'w');
fig.Position(3:4) = [1000, min(150 + 170 * np, 1000)];
tl = tiledlayout(fig, np, 1, 'TileSpacing', 'tight', 'Padding', 'compact');
title(tl, strjoin(files, newline), 'Interpreter', 'none', 'FontSize', 8);
ax = gobjects(np, 1);
for p = 1:np
    ax(p) = nexttile(tl);
    hold(ax(p), 'on');
    hs = gobjects(0);
    lg = {};
    for k = 1:numel(logs)
        L = logs{k};
        if isfield(L, 'time_s')
            t = L.time_s;
        else
            t = (0:numel(L.(panels{p, 2}{1})) - 1)';
        end
        for c = panels{p, 2}
            if ~isfield(L, c{1}) || ~isnumeric(L.(c{1}))
                continue
            end
            if numel(logs) > 1
                % ファイルごとに線の種類を変える(色は列ごと)
                styles = {'-', '--', ':', '-.'};
                h = plot(ax(p), t, L.(c{1}), styles{mod(k - 1, 4) + 1}, 'LineWidth', 0.9);
                lg{end+1} = sprintf('%s: %s', labels{k}, c{1}); %#ok<AGROW>
            else
                h = plot(ax(p), t, L.(c{1}), 'LineWidth', 0.9);
                lg{end+1} = c{1}; %#ok<AGROW>
            end
            hs(end+1) = h; %#ok<AGROW>
        end
        if numel(logs) > 1
            ax(p).ColorOrderIndex = 1;  % 次のファイルも同じ列は同じ色
        end
    end
    ylabel(ax(p), panels{p, 1});
    grid(ax(p), 'on');
    if ~isempty(hs)
        legend(ax(p), hs, lg, 'Location', 'northeast', 'Interpreter', 'none', 'FontSize', 7, ...
            'NumColumns', min(numel(hs), 5));
    end
    if p < np
        ax(p).XTickLabel = [];
    end
end
xlabel(ax(end), 'time [s]');
linkaxes(ax, 'x');
if nargout
    varargout{1} = fig;
end
end
