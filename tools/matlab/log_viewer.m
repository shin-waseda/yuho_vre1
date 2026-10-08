function log_viewer(path)
%LOG_VIEWER yuho のログ(SD の .bin、get_log.py が作った .csv)を見る GUI(tools/log_viewer.py の MATLAB 版)。
%   - 左: 列を選ぶ(クリックで付け外し)。同じ種類の列(速度・角度・壁センサーなど)は同じ段に描く。
%   - 真ん中: 時系列のグラフ。軸の右上のツールバーで拡大・移動(段の横軸はそろえて動く)。
%     イベントの所に縦線を引き、見えているイベントが少ないときは名前も書く。
%     グラフをクリックすると、その時刻にカーソルを置く(ツールバーの拡大・移動を使っていないとき)。
%   - 右上: イベントの一覧(ev_text)。クリックするとその時刻へ飛ぶ(今の拡大の幅のまま真ん中に来る)。
%     「絞る」に名前(正規表現)を書くと、その名前のイベントだけを一覧と縦線に出す。← → キーで前後のイベントへ。
%   - 右下: 迷路の上の軌道(車輪の距離とジャイロの向きで作る)。STEP イベントで読んだ壁を描く。
%     カーソルの時刻の位置と向きを出す。軌道をクリックすると、その時刻へ飛ぶ。
%     スタート区画の真ん中を、尻当て(SETPOS)が2回終わった所(なければ最初の STEP、それもなければ記録の始め)とする。
%     「STEP で合わせ直す」を付けると、STEP のたびに位置と向きを区画の境界(止まっていれば真ん中)に合わせ直す。
%   - 上: 再生(カーソルの時刻を実際の時間 × 速さで進める)。スペースキーでも再生・停止。
%
%   log_viewer                                   % 起動するとログを選ぶ画面が出る
%   log_viewer('logs/search/search_0006.bin')    % 開くログを決めて起動

S = 180.0;               % 区画の大きさ [mm]
LABEL_MAX = 40;          % 見えているイベントがこれ以下なら、縦線に名前を書く
PLAY_SPEEDS = {'0.05', '0.1', '0.25', '0.5', '1', '2', '4'};
PLAY_FRAME_S = 0.04;     % 再生で絵を描き直す間隔
MAP_CELLS_EVENT = 45;    % LOG_EV_MAP_CELLS(機体の app/log_event.h)
EV_COLOR = [0.58 0.40 0.74];
[GROUPS, DEFAULT_COLUMNS, HIDDEN_COLUMNS] = yc.plot_groups();

params = yc.read_params();
MAP_SIZE = 16;
if isfield(params, 'MAZE_SIZE'), MAP_SIZE = params.MAZE_SIZE; end

L = [];                  % ログ(列名 → 列ベクトル)
t = [];
events = struct('idx', zeros(0, 1), 't', zeros(0, 1), 'txt', strings(0, 1));  % 全部
shown = events;          % 絞った後
cursor_t = [];
path_xy = [];            % 迷路の上の軌道 [x y 向き(deg)]
axs = gobjects(0);
cursor_lines = gobjects(0);
label_texts = gobjects(0);
xlim_listener = [];
last_dir = fullfile(yc.root(), 'logs');
play_timer = [];
play_wall = [];
maze = struct();         % 迷路の図の線
step_walls = struct('row', zeros(0, 1), 'seg', zeros(0, 4), 'exists', false(0, 1));
map_cells = [];
maze_region = [0 0 1 1];
step_count_shown = -1;

% ---- 画面 ----
fig = uifigure('Name', 'yuho log viewer', 'Position', [20 20 1600 950], ...
    'WindowKeyPressFcn', @on_key, 'CloseRequestFcn', @(~, ~) on_close());
main = uigridlayout(fig, [3 1]);
main.RowHeight = {28, '1x', 20};
main.Padding = [4 4 4 4];
main.RowSpacing = 4;

top = uigridlayout(main, [1 7]);
top.ColumnWidth = {60, 80, 45, 60, 300, 130, '1x'};
top.Padding = [0 0 0 0];
uibutton(top, 'Text', '開く', 'ButtonPushedFcn', @(~, ~) open_dialog());
play_btn = uibutton(top, 'Text', '▶ 再生', 'ButtonPushedFcn', @(~, ~) toggle_play());
uilabel(top, 'Text', '速さ ×', 'HorizontalAlignment', 'right');
speed_dd = uidropdown(top, 'Items', PLAY_SPEEDS, 'Value', '1');
seek = uislider(top, 'Limits', [0 1], 'MajorTicks', [], 'MinorTicks', [], ...
    'ValueChangingFcn', @(~, e) on_seek(e.Value), 'ValueChangedFcn', @(s, ~) on_seek(s.Value));
seek_label = uilabel(top, 'Text', '', 'FontName', 'Consolas');
file_label = uilabel(top, 'Text', '(ログを開いてください)');

body = uigridlayout(main, [1 3]);
body.ColumnWidth = {170, '3x', '2x'};
body.Padding = [0 0 0 0];

% ---- 左: 列 ----
left = uigridlayout(body, [5 1]);
left.RowHeight = {20, '1x', 24, 24, 24};
left.Padding = [0 0 0 0];
uilabel(left, 'Text', '列 (クリックで付け外し)');
col_tree = uitree(left, 'checkbox', 'CheckedNodesChangedFcn', @(~, ~) redraw_plots(), ...
    'SelectionChangedFcn', @(~, e) on_col_click(e));
group_dd = uidropdown(left, 'Items', GROUPS(:, 1)');
uibutton(left, 'Text', 'この組を足す', 'ButtonPushedFcn', @(~, ~) add_group());
uibutton(left, 'Text', '全部外す', 'ButtonPushedFcn', @(~, ~) clear_columns());

% ---- 真ん中: グラフ ----
mid = uipanel(body, 'BorderType', 'none', 'BackgroundColor', 'w');

% ---- 右: イベントと迷路 ----
right = uigridlayout(body, [2 1]);
right.RowHeight = {'1x', '2x'};
right.Padding = [0 0 0 0];
evg = uigridlayout(right, [2 1]);
evg.RowHeight = {24, '1x'};
evg.Padding = [0 0 0 0];
frow = uigridlayout(evg, [1 5]);
frow.ColumnWidth = {30, '1x', 30, '1x', 50};
frow.Padding = [0 0 0 0];
uilabel(frow, 'Text', '絞る');
ev_filter = uieditfield(frow, 'text', 'ValueChangedFcn', @(~, ~) apply_event_filter(true));
uilabel(frow, 'Text', '除く');
ev_exclude = uieditfield(frow, 'text', 'Value', 'STEP_INFO', 'ValueChangedFcn', @(~, ~) apply_event_filter(true));
uibutton(frow, 'Text', '適用', 'ButtonPushedFcn', @(~, ~) apply_event_filter(true));
ev_list = uilistbox(evg, 'Items', {}, 'FontName', 'Consolas', 'ValueChangedFcn', @(s, ~) on_event_select(s.Value));

mzg = uigridlayout(right, [2 1]);
mzg.RowHeight = {24, '1x'};
mzg.Padding = [0 0 0 0];
mrow = uigridlayout(mzg, [1 3]);
mrow.ColumnWidth = {'fit', 'fit', 'fit'};
mrow.Padding = [0 0 0 0];
% 壁: 読んだ壁(赤 = ある、緑の破線 = ない。カーソルの時刻まで)、機体の地図(灰 = ある、点線 = 未知)
reanchor_cb = uicheckbox(mrow, 'Text', 'STEP で合わせ直す', 'ValueChangedFcn', @(~, ~) rebuild_maze());
steps_cb = uicheckbox(mrow, 'Text', '読んだ壁', 'Value', true, 'ValueChangedFcn', @(~, ~) on_layer_toggle());
map_cb = uicheckbox(mrow, 'Text', '機体の地図', 'Value', true, 'ValueChangedFcn', @(~, ~) on_layer_toggle());
max_ = uiaxes(mzg);
max_.ButtonDownFcn = @on_maze_click;

status = uilabel(main, 'Text', '', 'FontName', 'Consolas');

if nargin >= 1 && ~isempty(path)
    p = char(path);
    if ~isfile(p) && isfile(fullfile(yc.root(), p))
        p = fullfile(yc.root(), p);
    end
    open_log(p);
else
    open_dialog();  % 起動したらすぐログを選ぶ(あとから「開く」で選び直せる)
end


% =====================================================================
% 読む
% =====================================================================
    function open_dialog()
        [f, d] = uigetfile({'*.bin;*.csv', 'log (*.bin, *.csv)'; '*.*', 'all'}, 'ログ', [last_dir filesep]);
        figure(fig);
        if isequal(f, 0), return; end
        last_dir = d(1:end-1);
        open_log(fullfile(d, f));
    end

    function open_log(p)
        try
            [Ln, names] = yc.load_log(p);
        catch ex
            uialert(fig, sprintf('%s: %s', p, ex.message), '開く');
            return
        end
        if ~isfield(Ln, 'time_s') || numel(Ln.time_s) < 2
            uialert(fig, 'time_s の列がない、または行が少ない', '開く');
            return
        end
        stop_play();
        L = Ln;
        t = L.time_s;
        seek.Limits = [t(1), t(end)];
        seek.Value = t(1);
        [events.idx, events.t, events.txt] = yc.find_events(L);
        dt = median(diff(t));
        file_label.Text = sprintf('%s  (%d 行, %.2f s, %.0f ms ごと, イベント %d 個)', p, numel(t), ...
            t(end) - t(1), dt * 1000, numel(events.idx));
        cols = names(~ismember(names, HIDDEN_COLUMNS));
        delete(col_tree.Children);
        nodes = matlab.ui.container.TreeNode.empty;
        for k = 1:numel(cols)
            nodes(end+1) = uitreenode(col_tree, 'Text', cols{k}, 'NodeData', cols{k}); %#ok<AGROW>
        end
        if ~isempty(nodes)
            col_tree.CheckedNodes = nodes(ismember(cols, DEFAULT_COLUMNS));
        end
        cursor_t = [];
        axs = gobjects(0);  % 前のログの拡大の幅は使わない
        apply_event_filter(false);
        redraw_plots();
        rebuild_maze();
        update_seek_label();
    end

% =====================================================================
% 列
% =====================================================================
    function cols = selected_columns()
        cols = {};
        if isempty(col_tree.CheckedNodes), return; end
        checked = {col_tree.CheckedNodes.NodeData};
        all_nodes = col_tree.Children;
        for k = 1:numel(all_nodes)
            if ismember(all_nodes(k).NodeData, checked)
                cols{end+1} = all_nodes(k).NodeData; %#ok<AGROW>
            end
        end
    end

    function on_col_click(e)
        % 名前をクリックしても付け外しする(チェックの箱だけでなく)
        node = e.SelectedNodes;
        if isempty(node), return; end
        checked = col_tree.CheckedNodes;
        if isempty(checked)
            col_tree.CheckedNodes = node;
        elseif any(checked == node)
            col_tree.CheckedNodes = checked(checked ~= node);
        else
            col_tree.CheckedNodes = [checked; node];
        end
        col_tree.SelectedNodes = [];
        redraw_plots();
    end

    function add_group()
        k = find(strcmp(GROUPS(:, 1), group_dd.Value), 1);
        names = GROUPS{k, 2};
        nodes = col_tree.Children;
        cur = col_tree.CheckedNodes;
        now_names = {};
        if ~isempty(cur), now_names = {cur.NodeData}; end
        col_tree.CheckedNodes = nodes(ismember({nodes.NodeData}, [now_names, names]));
        redraw_plots();
    end

    function clear_columns()
        col_tree.CheckedNodes = [];
        redraw_plots();
    end

% =====================================================================
% イベント
% =====================================================================
    function apply_event_filter(redraw)
        inc = strtrim(ev_filter.Value);
        exc = strtrim(ev_exclude.Value);
        try
            ok = true(numel(events.idx), 1);
            if ~isempty(inc)
                ok = ok & ~cellfun(@isempty, regexpi(cellstr(events.txt), inc, 'once'));
            end
            if ~isempty(exc)
                head = cellstr(extractBefore(events.txt + " ", " "));
                ok = ok & cellfun(@isempty, regexpi(head, exc, 'once'));
            end
        catch ex
            uialert(fig, sprintf('正規表現の誤り: %s', ex.message), '絞る');
            return
        end
        shown.idx = events.idx(ok); shown.t = events.t(ok); shown.txt = events.txt(ok);
        if isempty(shown.idx)
            ev_list.Items = {};
        else
            ev_list.Items = cellstr(compose("%8.3f  %s", shown.t, shown.txt));
        end
        ev_list.ItemsData = 1:numel(shown.idx);
        try  % まだ選ばない
            ev_list.Value = {};
        catch
        end
        if redraw
            redraw_plots();
        end
    end

    function on_event_select(i)
        if ~isempty(i)
            set_cursor(shown.t(i), true, true, false);
        end
    end

    function step_event(d)
        if isempty(shown.idx), return; end
        i = ev_list.Value;
        if ~isempty(i) && isnumeric(i)
            i = i + d;
        else
            if isempty(cursor_t), c = t(1); else, c = cursor_t; end
            i = sum(shown.t < c) + (d > 0);  % 進むならカーソルの後の最初、戻るならその前
        end
        i = max(1, min(numel(shown.idx), i));
        ev_list.Value = i;
        try
            scroll(ev_list, i);
        catch
        end
        set_cursor(shown.t(i), true, true, false);
    end

% =====================================================================
% グラフ
% =====================================================================
    function redraw_plots()
        if isempty(L), return; end
        old_xlim = [];
        if ~isempty(axs) && isvalid(axs(1))
            old_xlim = axs(1).XLim;
        end
        delete(xlim_listener);
        xlim_listener = [];
        delete(mid.Children);
        axs = gobjects(0);
        cursor_lines = gobjects(0);
        label_texts = gobjects(0);
        cols = selected_columns();
        panels = cell(0, 2);
        used = {};
        for g = 1:size(GROUPS, 1)
            sel = GROUPS{g, 2}(ismember(GROUPS{g, 2}, cols));
            if ~isempty(sel)
                panels(end+1, :) = {GROUPS{g, 1}, sel}; %#ok<AGROW>
                used = [used, sel]; %#ok<AGROW>
            end
        end
        for c = cols
            if ~ismember(c{1}, used)
                panels(end+1, :) = {c{1}, c}; %#ok<AGROW>
            end
        end
        if isempty(panels), return; end
        np = size(panels, 1);
        tl = tiledlayout(mid, np, 1, 'TileSpacing', 'tight', 'Padding', 'compact');
        ev_t = shown.t(:)';
        for k = 1:np
            ax = nexttile(tl);
            hold(ax, 'on');
            hs = gobjects(0);
            for c = panels{k, 2}
                hs(end+1) = plot(ax, t, L.(c{1}), 'LineWidth', 0.9, 'PickableParts', 'none'); %#ok<AGROW>
            end
            ylabel(ax, panels{k, 1}, 'FontSize', 8);
            legend(ax, hs, panels{k, 2}, 'Location', 'northeast', 'FontSize', 7, 'Interpreter', 'none', ...
                'NumColumns', min(numel(hs), 5), 'AutoUpdate', 'off');
            grid(ax, 'on');
            ax.FontSize = 7;
            ax.XLim = [t(1), t(end)];
            if ~isempty(ev_t)
                yl = ax.YLim;
                ax.YLimMode = 'manual';
                X = [ev_t; ev_t; nan(size(ev_t))];
                Y = repmat([yl(1); yl(2); nan], 1, numel(ev_t));
                line(ax, X(:), Y(:), 'Color', 0.35 * EV_COLOR + 0.65, 'LineWidth', 0.6, 'PickableParts', 'none');
            end
            cursor_lines(k) = xline(ax, t(1), 'Color', 'r', 'LineWidth', 1.0, 'Visible', 'off', 'HitTest', 'off');
            ax.ButtonDownFcn = @on_plot_click;
            try
                ax.Interactions = zoomInteraction('Dimensions', 'x');  % ホイールで横に拡大
            catch
            end
            if k < np
                ax.XTickLabel = [];
            end
            axs(k) = ax;
        end
        xlabel(axs(end), 'time [s]');
        if numel(axs) > 1
            linkaxes(axs, 'x');
        end
        if ~isempty(old_xlim)
            axs(1).XLim = old_xlim;
        end
        xlim_listener = addlistener(axs(1), 'XLim', 'PostSet', @(~, ~) update_labels());
        update_labels();
        update_cursor_lines();
    end

    function update_labels()
        delete(label_texts(isgraphics(label_texts)));
        label_texts = gobjects(0);
        if isempty(axs) || ~isvalid(axs(1)), return; end
        ax = axs(1);
        x = ax.XLim;
        vis = shown.t >= x(1) & shown.t <= x(2);
        if nnz(vis) > LABEL_MAX, return; end
        tv = shown.t(vis);
        heads = extractBefore(shown.txt(vis) + " ", " ");
        [ut, ~, j] = unique(tv);  % 同じ行のイベントは1つの名前にまとめる
        y = ax.YLim(2);
        for k = 1:numel(ut)
            label_texts(end+1) = text(ax, ut(k), y, strjoin(heads(j == k), '/'), 'Rotation', 90, ...
                'FontSize', 6, 'Color', EV_COLOR, 'HorizontalAlignment', 'left', ...
                'VerticalAlignment', 'middle', 'Clipping', 'off', 'Interpreter', 'none', 'PickableParts', 'none'); %#ok<AGROW>
        end
    end

    function update_cursor_lines()
        if isempty(cursor_t), return; end
        for k = 1:numel(cursor_lines)
            if isvalid(cursor_lines(k))
                cursor_lines(k).Value = cursor_t;
                cursor_lines(k).Visible = 'on';
            end
        end
    end

    function on_plot_click(~, e)
        if e.Button ~= 1, return; end
        set_cursor(e.IntersectionPoint(1), false, false, false);
        figure(fig);  % キーを受けるように
    end

% =====================================================================
% 再生
% =====================================================================
    function on_key(~, e)
        obj = fig.CurrentObject;
        if isa(obj, 'matlab.ui.control.EditField')
            return  % 絞る・除くに文字を打っているとき
        end
        switch e.Key
            case 'leftarrow'
                step_event(-1);
            case 'rightarrow'
                step_event(+1);
            case 'space'
                toggle_play();
        end
    end

    function toggle_play()
        if ~isempty(play_timer) && isvalid(play_timer) && strcmp(play_timer.Running, 'on')
            stop_play();
        else
            start_play();
        end
    end

    function start_play()
        if isempty(L), return; end
        if isempty(cursor_t) || cursor_t >= t(end)
            cursor_t = t(1);  % 終わりまで行っていたら最初から
        end
        play_btn.Text = '⏸ 停止';
        play_wall = tic;
        if isempty(play_timer) || ~isvalid(play_timer)
            play_timer = timer('ExecutionMode', 'fixedSpacing', 'Period', PLAY_FRAME_S, ...
                'BusyMode', 'drop', 'TimerFcn', @(~, ~) play_tick());
        end
        start(play_timer);
    end

    function stop_play()
        if ~isempty(play_timer) && isvalid(play_timer)
            stop(play_timer);
        end
        if isvalid(play_btn)
            play_btn.Text = '▶ 再生';
        end
    end

    function play_tick()
        try
            if isempty(L) || ~isvalid(fig), return; end
            speed = str2double(speed_dd.Value);
            if isnan(speed), speed = 1.0; end
            tn = cursor_t + toc(play_wall) * speed;  % 描くのが遅ければ、実際の時間に合わせてコマを飛ばす
            play_wall = tic;
            if tn >= t(end)
                set_cursor(t(end), false, false, true);
                stop_play();
            else
                set_cursor(tn, false, false, true);
            end
            drawnow limitrate
        catch ex
            stop_play();
            warning('log_viewer:play', '%s', ex.message);
        end
    end

    function on_seek(v)
        if isempty(L), return; end
        set_cursor(v, false, false, true);
        play_wall = tic;
    end

    function update_seek_label()
        if isempty(cursor_t), c = t(1); else, c = cursor_t; end
        seek_label.Text = sprintf('%7.3f / %.2f s', c, t(end));
    end

    function set_cursor(tc, center, from_list, follow)
        if isempty(L), return; end
        cursor_t = double(tc);
        full = t(end) - t(1);
        if follow && ~isempty(axs) && isvalid(axs(1))
            % 拡大しているときは、カーソルが右の端近くまで来たら、左の端近くへ来るように送る
            x = axs(1).XLim;
            w = x(2) - x(1);
            if w < full * 0.999 && ~(x(1) <= cursor_t && cursor_t <= x(1) + 0.9 * w)
                lo = min(max(cursor_t - 0.1 * w, t(1)), t(end) - w);
                axs(1).XLim = [lo, lo + w];
            end
        end
        seek.Value = min(max(cursor_t, seek.Limits(1)), seek.Limits(2));
        update_seek_label();
        if center && ~isempty(axs) && isvalid(axs(1))
            x = axs(1).XLim;
            w = x(2) - x(1);
            if w >= full * 0.999  % まだ拡大していなければ、前後 1 秒を見る
                w = min(2.0, full);
            end
            lo = min(max(cursor_t - w / 2, t(1)), t(end) - w);  % 記録の外を見せない
            axs(1).XLim = [lo, lo + w];
        end
        update_cursor_lines();
        update_maze_marker();
        update_status();
        if ~from_list && ~isempty(shown.idx)
            % 一覧で、カーソルに一番近い(前の)イベントを選ぶ
            i = max(1, sum(shown.t <= cursor_t));
            if ~isequal(ev_list.Value, i)
                ev_list.Value = i;
                try
                    scroll(ev_list, i);
                catch
                end
            end
        end
    end

    function i = cursor_row()
        % カーソルの時刻の行(その時刻以後の最初の行)
        i = find(t >= cursor_t, 1);
        if isempty(i), i = numel(t); end
    end

    function update_status()
        i = cursor_row();
        parts = {sprintf('t=%.3fs', t(i))};
        for c = selected_columns()
            parts{end+1} = sprintf('%s=%.4g', c{1}, L.(c{1})(i)); %#ok<AGROW>
        end
        if isfield(L, 'ev_text') && L.ev_text(i) ~= ""
            parts{end+1} = sprintf('[%s]', L.ev_text(i));
        end
        status.Text = strjoin(parts, '  ');
    end

% =====================================================================
% 迷路
% =====================================================================
    function [i0, name] = find_anchor()
        % スタート区画の真ん中・北向きとする行
        [idx, ~, txt] = yc.find_events(L, 'SETPOS');
        ends = [];
        for k = 1:numel(idx)
            v = yc.event_values(txt(k));
            if isfield(v, 'phase') && v.phase >= 0.5
                ends(end+1) = idx(k); %#ok<AGROW>
            end
        end
        if numel(ends) >= 2
            i0 = ends(2); name = '尻当て2回目の終わり';
            return
        end
        idx = yc.find_events(L, 'STEP');
        if ~isempty(idx)
            i0 = idx(1); name = '最初の STEP';
            return
        end
        i0 = 1; name = '記録の始め';
    end

    function rebuild_maze()
        ax = max_;
        cla(ax);
        hold(ax, 'on');
        path_xy = [];
        if isempty(L), return; end
        if ~isfield(L, 'dist') || ~isfield(L, 'angle')
            title(ax, 'dist と angle の列がないので軌道は描けない', 'FontSize', 9);
            return
        end
        n = numel(t);
        ang = L.angle;
        ds = [0; diff(L.dist)];
        ds(ds < -5.0) = 0.0;  % 制御を有効にし直して距離が 0 に戻った所
        [i0, anchor_name] = find_anchor();
        [sidx, ~, stxt] = yc.find_events(L, 'STEP');
        svals = arrayfun(@(s) yc.event_values(s), stxt, 'UniformOutput', false);

        % 合わせる点: [行, x, y, 向き(deg)]
        anchors = [i0, 0.5 * S, 0.5 * S, 90.0];
        if reanchor_cb.Value
            for k = 1:numel(sidx)
                i = sidx(k); v = svals{k};
                if i <= i0 || ~all(isfield(v, {'x', 'y', 'heading'})), continue; end
                h = round(v.heading);
                cx = (v.x + 0.5) * S; cy = (v.y + 0.5) * S;
                if isfield(L, 'target') && abs(L.target(i)) > 10.0
                    % 境界で壁を読んだ: 区画の真ん中から、来た向きへ半区画戻った所
                    dv = dir_vec(h);
                    cx = cx - dv(1) * 0.5 * S;
                    cy = cy - dv(2) * 0.5 * S;
                end
                anchors(end+1, :) = [i, cx, cy, heading_deg(h)]; %#ok<AGROW>
            end
        end

        x = nan(n, 1); y = nan(n, 1); th = nan(n, 1);
        % 合わせる点から先へ進める(最初の点より前は、後ろへ戻して作る)
        bounds = [anchors(:, 1); n + 1];
        for k = 1:size(anchors, 1)
            ia = anchors(k, 1); ib = bounds(k + 1);
            seg = (ia:ib - 1)';
            if isempty(seg), continue; end
            th(seg) = ang(seg) + (anchors(k, 4) - ang(ia));
            s2 = seg(2:end);
            x(seg) = anchors(k, 2) + [0; cumsum(ds(s2) .* cosd(th(s2)))];
            y(seg) = anchors(k, 3) + [0; cumsum(ds(s2) .* sind(th(s2)))];
        end
        if i0 > 1
            off = anchors(1, 4) - ang(i0);
            th(1:i0 - 1) = ang(1:i0 - 1) + off;
            r = (2:i0)';
            x(1:i0 - 1) = x(i0) - flipud(cumsum(flipud(ds(r) .* cosd(th(r)))));
            y(1:i0 - 1) = y(i0) - flipud(cumsum(flipud(ds(r) .* sind(th(r)))));
        end
        path_xy = [x, y, th];

        % 区画の大きさ: 軌道と STEP が入るだけ
        cells_x = [cellfun(@(v) getd(v, 'x'), svals(:)); 0];
        cells_y = [cellfun(@(v) getd(v, 'y'), svals(:)); 0];
        nx_ = max(max(cells_x), floor(max(x) / S)) + 1;
        ny_ = max(max(cells_y), floor(max(y) / S)) + 1;
        mx0 = min(0, floor(min(x) / S));
        my0 = min(0, floor(min(y) / S));
        xline(ax, (mx0:nx_) * S, 'Color', [0.9 0.9 0.9], 'LineWidth', 0.6, 'HitTest', 'off');
        yline(ax, (my0:ny_) * S, 'Color', [0.9 0.9 0.9], 'LineWidth', 0.6, 'HitTest', 'off');
        [GX, GY] = ndgrid((mx0:nx_) * S, (my0:ny_) * S);
        px = GX(:)'; py = GY(:)';
        patch(ax, [px - 6; px + 6; px + 6; px - 6], [py - 6; py - 6; py + 6; py + 6], [0.3 0.3 0.3], ...
            'EdgeColor', 'none', 'PickableParts', 'none');

        % 壁(機体の地図と、カーソルの時刻までに STEP で読んだ壁)。描く範囲は軌道・STEP・地図で分かった区画
        maze_region = [mx0, my0, nx_, ny_];
        map_cells = parse_map_cells();
        step_walls = parse_step_walls(sidx, svals);
        maze.map_known = line(ax, nan, nan, 'Color', [0.25 0.25 0.25], 'LineWidth', 3, 'PickableParts', 'none');
        maze.map_unknown = line(ax, nan, nan, 'Color', [0.6 0.6 0.6], 'LineWidth', 0.8, 'LineStyle', ':', 'PickableParts', 'none');
        maze.step_wall = line(ax, nan, nan, 'Color', [0.84 0.15 0.16], 'LineWidth', 3, 'PickableParts', 'none');
        maze.step_open = line(ax, nan, nan, 'Color', 0.7 * [0.17 0.63 0.17] + 0.3, 'LineWidth', 1.0, 'LineStyle', '--', 'PickableParts', 'none');
        update_map_layer();
        step_count_shown = -1;
        update_step_layer(true);

        plot(ax, x, y, 'LineWidth', 1.0, 'Color', [0.12 0.47 0.71], 'PickableParts', 'none');
        plot(ax, x(i0), y(i0), 'o', 'MarkerSize', 5, 'MarkerFaceColor', [0.17 0.63 0.17], ...
            'MarkerEdgeColor', [0.17 0.63 0.17], 'PickableParts', 'none');
        axis(ax, 'equal');
        xlim(ax, [mx0 * S - 20, nx_ * S + 20]);
        ylim(ax, [my0 * S - 20, ny_ * S + 20]);
        ax.FontSize = 7;
        ttl = sprintf('軌道 (車輪+ジャイロ、基準: %s)', anchor_name);
        if isempty(map_cells)
            ttl = [ttl, newline, '機体の地図はログにない'];
        end
        title(ax, ttl, 'FontSize', 9, 'FontWeight', 'normal');
        % カーソルの位置と向き
        maze.dot = plot(ax, nan, nan, 'o', 'Color', 'r', 'MarkerFaceColor', 'r', 'MarkerSize', 7, 'PickableParts', 'none');
        maze.dir = plot(ax, nan, nan, '-', 'Color', 'r', 'LineWidth', 2, 'PickableParts', 'none');
        update_maze_marker();
    end

    function cells = parse_map_cells()
        % LOG_EV_MAP_CELLS から機体の地図(MAP_SIZE × MAP_SIZE、1区画1バイト、来ていない区画は NaN)を作る。
        % 同じ区画が何度も出たら、後のもの(走りの終わりに近いもの)を使う。
        cells = [];
        if ~all(isfield(L, {'ev', 'ev_a', 'ev_b', 'ev_c', 'ev_d', 'ev_e'})), return; end
        rows = find(round(L.ev) == MAP_CELLS_EVENT);
        if isempty(rows), return; end
        cells = nan(MAP_SIZE, MAP_SIZE);  % cells(x+1, y+1)
        wcols = {'ev_b', 'ev_c', 'ev_d', 'ev_e'};
        for r = rows'
            index = round(L.ev_a(r));
            for k = 1:4
                w = round(L.(wcols{k})(r));
                bytes = [bitand(w, 255), bitand(bitshift(w, -8), 255)];
                for j = 1:2
                    c = index + 2 * (k - 1) + (j - 1);
                    if c >= 0 && c < MAP_SIZE * MAP_SIZE
                        cells(mod(c, MAP_SIZE) + 1, floor(c / MAP_SIZE) + 1) = bytes(j);
                    end
                end
            end
        end
    end

    function sw = parse_step_walls(sidx, svals)
        % STEP で読んだ壁を [行, 線分, ある?] にする(前・右・左。bit0 前, bit1 右, bit2 左)
        sw = struct('row', zeros(0, 1), 'seg', zeros(0, 4), 'exists', false(0, 1));
        for k = 1:numel(sidx)
            v = svals{k};
            if ~all(isfield(v, {'x', 'y', 'heading', 'walls'})), continue; end
            cx = round(v.x); cy = round(v.y); h = round(v.heading); w = round(v.walls);
            bits = [1, 2, 4]; turns = [0, 1, -1];
            for b = 1:3
                sw.row(end+1, 1) = sidx(k);
                sw.seg(end+1, :) = wall_segment(cx, cy, mod(h + turns(b), 4));
                sw.exists(end+1, 1) = bitand(w, bits(b)) ~= 0;
            end
        end
    end

    function seg = wall_segment(cx, cy, d)
        % 区画 (cx, cy) の d の向き(0 北 1 東 2 南 3 西)の壁の線分 [x1 y1 x2 y2]
        x0 = cx * S; y0 = cy * S;
        switch d
            case 0, seg = [x0, y0 + S, x0 + S, y0 + S];
            case 1, seg = [x0 + S, y0, x0 + S, y0 + S];
            case 2, seg = [x0, y0, x0 + S, y0];
            otherwise, seg = [x0, y0, x0, y0 + S];
        end
    end

    function update_map_layer()
        known = zeros(0, 4); unknown = zeros(0, 4);
        if ~isempty(map_cells) && map_cb.Value
            r = maze_region;
            for cx = max(r(1), 0):min(r(3), MAP_SIZE) - 1
                for cy = max(r(2), 0):min(r(4), MAP_SIZE) - 1
                    byte = map_cells(cx + 1, cy + 1);
                    if isnan(byte), continue; end
                    % 北0x8 東0x4 南0x2 西0x1。下位4bit 探索用(未知は「ない」)、上位4bit 最短用(未知は「ある」)
                    dbits = [8, 4, 2, 1];
                    for d = 0:3
                        lo = bitand(byte, dbits(d + 1)) ~= 0;
                        hi = bitand(byte, dbits(d + 1) * 16) ~= 0;
                        if lo && hi
                            known(end+1, :) = wall_segment(cx, cy, d); %#ok<AGROW>
                        elseif hi && ~lo
                            unknown(end+1, :) = wall_segment(cx, cy, d); %#ok<AGROW>
                        end
                    end
                end
            end
        end
        set_segments(maze.map_known, known);
        set_segments(maze.map_unknown, unknown);
    end

    function changed = update_step_layer(force)
        % カーソルの時刻までに読んだ壁を描く(カーソルがなければ全部)
        if ~steps_cb.Value
            n = 0;
        elseif isempty(cursor_t)
            n = numel(step_walls.row);
        else
            n = nnz(step_walls.row <= nnz(t <= cursor_t));
        end
        changed = false;
        if n == step_count_shown && ~force, return; end
        step_count_shown = n;
        seg = step_walls.seg(1:n, :);
        ex = step_walls.exists(1:n);
        % 同じ壁を何度も読んだら、後に読んだ結果を使う(端点の順をそろえて同じ壁を見つける)
        key = seg;
        sw = seg(:, 1) > seg(:, 3) | (seg(:, 1) == seg(:, 3) & seg(:, 2) > seg(:, 4));
        key(sw, :) = seg(sw, [3 4 1 2]);
        [~, last] = unique(key, 'rows', 'last');
        seg = seg(last, :); ex = ex(last);
        set_segments(maze.step_wall, seg(ex, :));
        set_segments(maze.step_open, seg(~ex, :));
        changed = true;
    end

    function on_layer_toggle()
        if isempty(path_xy), return; end
        update_map_layer();
        update_step_layer(true);
    end

    function update_maze_marker()
        if isempty(path_xy) || ~isfield(maze, 'dot') || ~isvalid(maze.dot), return; end
        update_step_layer(false);
        if isempty(cursor_t)
            set(maze.dot, 'XData', nan, 'YData', nan);
            set(maze.dir, 'XData', nan, 'YData', nan);
            return
        end
        i = cursor_row();
        x = path_xy(i, 1); y = path_xy(i, 2); a = path_xy(i, 3);
        set(maze.dot, 'XData', x, 'YData', y);
        set(maze.dir, 'XData', [x, x + 50 * cosd(a)], 'YData', [y, y + 50 * sind(a)]);
    end

    function on_maze_click(~, e)
        if isempty(path_xy) || e.Button ~= 1, return; end
        p = e.IntersectionPoint;
        d = hypot(path_xy(:, 1) - p(1), path_xy(:, 2) - p(2));
        if all(isnan(d)), return; end
        [~, i] = min(d);
        set_cursor(t(i), true, false, false);
        figure(fig);
    end

    function on_close()
        if ~isempty(play_timer) && isvalid(play_timer)
            stop(play_timer);  % 再生の次のコマの予約を消してから閉じる
            delete(play_timer);
        end
        delete(xlim_listener);
        delete(fig);
    end
end


% =====================================================================
function h = heading_deg(d)
% 迷路の向き(0 北 1 東 2 南 3 西)→ 数学の角度(+x が 0°、+y が 90°)
h = 90.0 - 90.0 * d;
end

function v = dir_vec(d)
v = round([cosd(heading_deg(d)), sind(heading_deg(d))]);
end

function v = getd(s, name)
if isfield(s, name)
    v = s.(name);
else
    v = 0;
end
end

function set_segments(h, seg)
% 線分 [x1 y1 x2 y2; ...] を NaN で区切った1本の線にする
if isempty(seg)
    set(h, 'XData', nan, 'YData', nan);
    return
end
n = size(seg, 1);
X = [seg(:, 1), seg(:, 3), nan(n, 1)]';
Y = [seg(:, 2), seg(:, 4), nan(n, 1)]';
set(h, 'XData', X(:), 'YData', Y(:));
end
