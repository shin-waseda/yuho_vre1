function turn_sim(files)
%TURN_SIM yuho のスラロームのシミュレータ(tools/turn_sim.py の MATLAB 版)。
%   - 機体と同じ計算: params.h を読み、logic/control/slalom.c と同じ台形・同じ前後のオフセット
%     (小回り 90°、大回り 90°、大回り 180°)で曲がる。
%   - スリップアングル: 進む向き = 機体の向き − β。β は 1次遅れ
%         dβ/dt = (K × v[m/s] × ω[rad/s] − β) / C        (C = 0 なら β = K v ω)
%     β は外(曲がる中心の反対)へずれる向きを + とする。
%   - 結果: 機体と同じ前後のオフセット、出口のずれ(前後・横)、柱との距離、
%     ずれを消す PRE/POST_ADJ(params.h に貼れる形)。
%   - 実機のログ(SLALOM テストの logs/slalom/*.csv / .bin)を重ねる: 車輪の距離とジャイロの向きで軌道を作る。
%     曲がった後の横の壁センサー(L, R)で見た横のずれとの差を「スリップで横にずれた量」とし、
%     それに合う K と C を探す(yc.fit_slip。格子で探した後、fminsearch で詰める)。
%
%   turn_sim                                     % MATLAB で tools/matlab をパスに入れて(または cd して)
%   turn_sim({'logs/slalom/turn_0001.bin', 'logs/slalom/turn_0002.bin'})   % ログを開いた状態で起動

kinds = yc.turn_kinds();
kind_keys = fieldnames(kinds)';
log_dir = fullfile(yc.root(), 'logs');

params = yc.read_params();
logs = yc.TurnLog.empty;
fit_texts = struct();   % 旋回の種類ごとの、合わせた結果
fit_b = struct();       % 旋回の種類ごとの、合わせた共通の横のずれ b [mm](右へ)
paste_text = '';

fig = uifigure('Name', 'yuho turn sim', 'Position', [40 40 1400 900]);
g = uigridlayout(fig, [1 2]);
g.ColumnWidth = {400, '1x'};
g.Padding = [4 4 4 4];
left = uigridlayout(g, [3 1]);
left.RowHeight = {'fit', 'fit', '1x'};
left.Padding = [0 0 0 0];

% ---- 旋回 ----
p1 = uipanel(left, 'Title', '旋回');
fields = {
    'v',        '並進の速さ [mm/s]'
    'omega',    '最高角速度 [dps]'
    'alpha',    '角加速度 [dps²]'
    'pre_adj',  'PRE_ADJ [mm]'
    'post_adj', 'POST_ADJ [mm]'
    'K',        'スリップ K [s²/m]'
    'C',        'スリップ C [s]'
    'width',    '機体の幅 [mm]'
    'extra',    '出口の直進 [mm]'
    };
nf = size(fields, 1);
g1 = uigridlayout(p1, [nf + 3, 2]);
g1.ColumnWidth = {'1x', 120};
g1.RowHeight = repmat({22}, 1, nf + 3);
g1.RowSpacing = 4;
uilabel(g1, 'Text', '種類');
kind_dd = uidropdown(g1, 'Items', cellfun(@(k) sprintf('%s (%s)', kinds.(k).label, k), kind_keys, 'UniformOutput', false), ...
    'ItemsData', kind_keys, 'Value', 's90', 'ValueChangedFcn', @(~, ~) load_from_params());
uilabel(g1, 'Text', '向き');
dir_dd = uidropdown(g1, 'Items', {'右', '左'}, 'ItemsData', {'R', 'L'}, 'Value', 'R', ...
    'ValueChangedFcn', @(~, ~) update());
ed = struct();
for r = 1:nf
    uilabel(g1, 'Text', fields{r, 2});
    ed.(fields{r, 1}) = uieditfield(g1, 'numeric', 'ValueDisplayFormat', '%.6g', ...
        'ValueChangedFcn', @(~, ~) update());
end
ed.K.ValueDisplayFormat = '%.4f';
ed.C.ValueDisplayFormat = '%.3f';
ed.width.Value = 86;
ed.extra.Value = 180;
uibutton(g1, 'Text', 'params.h から読む', 'ButtonPushedFcn', @(~, ~) reload_params());
uibutton(g1, 'Text', '計算', 'ButtonPushedFcn', @(~, ~) update());

% ---- 実機のログ ----
p2 = uipanel(left, 'Title', '実機のログ (SLALOM テスト)');
g2 = uigridlayout(p2, [7 1]);
g2.RowHeight = {24, 110, 24, 24, 22, 22, 26};
g2.RowSpacing = 4;
brow = uigridlayout(g2, [1 3]);
brow.Padding = [0 0 0 0];
uibutton(brow, 'Text', '開く', 'ButtonPushedFcn', @(~, ~) open_logs());
uibutton(brow, 'Text', '外す', 'ButtonPushedFcn', @(~, ~) remove_log());
uibutton(brow, 'Text', '全部外す', 'ButtonPushedFcn', @(~, ~) clear_logs());
log_list = uilistbox(g2, 'Items', {}, 'Multiselect', 'on');
srow = uigridlayout(g2, [1 2]);
srow.Padding = [0 0 0 0];
srow.ColumnWidth = {'1x', 80};
uilabel(srow, 'Text', '横センサーの傾き [AD/mm]');
slope_ed = uieditfield(srow, 'numeric', 'Value', 8.0, 'ValueDisplayFormat', '%.1f', ...
    'ValueChangedFcn', @(~, ~) update());
uibutton(g2, 'Text', '傾きを探索のログから見積もる', 'ButtonPushedFcn', @(~, ~) estimate_slope());
bias_cb = uicheckbox(g2, 'Text', '右・左で共通の横のずれ b も合わせる', 'Value', true, ...
    'Tooltip', ['WALL_REF・置き方で、どちらに曲がっても同じ側へずれて見える分。' newline '右・左の両方のログが要る']);
refine_cb = uicheckbox(g2, 'Text', '格子の後 fminsearch で細かく詰める', 'Value', true, ...
    'Tooltip', '外すと Python 版(turn_sim.py)と同じ格子だけの結果');
uibutton(g2, 'Text', 'K と C をログに合わせる', 'ButtonPushedFcn', @(~, ~) fit());

% ---- 結果 ----
p3 = uipanel(left, 'Title', '結果');
g3 = uigridlayout(p3, [2 1]);
g3.RowHeight = {'1x', 26};
result = uitextarea(g3, 'Editable', 'off', 'FontName', 'MS Gothic', 'FontSize', 12);
uibutton(g3, 'Text', 'params.h の行をコピー', 'ButtonPushedFcn', @(~, ~) clipboard('copy', paste_text));

% ---- 図 ----
pr = uipanel(g, 'BorderType', 'none', 'BackgroundColor', 'w');
tl = tiledlayout(pr, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_map = nexttile(tl, [2 1]);
ax_lat = nexttile(tl);

load_from_params();
if nargin >= 1 && ~isempty(files)
    add_logs(cellstr(files));
end


% =====================================================================
    function reload_params()
        params = yc.read_params();
        load_from_params();
    end

    function load_from_params()
        kind = kind_dd.Value;
        tp = yc.turn_params_from_h(params, kind);
        ed.v.Value = tp.v;
        if ~isnan(tp.omega)
            ed.omega.Value = tp.omega;
        end
        ed.alpha.Value = tp.alpha;
        ed.pre_adj.Value = tp.pre_adj;
        ed.post_adj.Value = tp.post_adj;
        ed.omega.Editable = ~strcmp(kind, 'l180');  % l180 は横が 1区画になる ω を求める
        update();
    end

    % ---- シミュレーション ----
    function [sim, omega, pre0, post0, shape] = simulate(K, C, pre_adj, post_adj)
        kind = kind_dd.Value;
        v = ed.v.Value; alpha = ed.alpha.Value;
        [omega, pre0, post0, shape] = yc.turn_offsets(kind, v, ed.omega.Value, alpha);
        sim = yc.simulate_turn(kind, v, omega, alpha, pre0 + pre_adj, post0 + post_adj, K, C, ed.extra.Value);
    end

    function [pre_adj, post_adj] = suggest_adj(K, C, pre_adj, post_adj)
        % スリップがあっても出口の線に乗るような PRE/POST_ADJ(前後の直進をずらすだけで直る分)
        kind = kind_dd.Value;
        for it = 1:3
            e = yc.exit_errors(kind, simulate(K, C, pre_adj, post_adj));
            if strcmp(kind, 'l180')
                d_pre = -(e.ymax - 180.0);   % 一番奥が 1区画先になるように
                d_post = -(e.along - d_pre); % 前を伸ばすと出口は手前に来る
            else
                d_pre = -e.lat_final;        % 前を伸ばすと出口の線が外へずれる
                d_post = -e.along;
            end
            pre_adj = pre_adj + d_pre;
            post_adj = post_adj + d_post;
        end
    end

    function update()
        kind = kind_dd.Value;
        kd = kinds.(kind);
        K = ed.K.Value; C = ed.C.Value;
        pre_adj = ed.pre_adj.Value; post_adj = ed.post_adj.Value;
        [sim, omega, pre0, post0, shape] = simulate(K, C, pre_adj, post_adj);
        ideal = simulate(0.0, 0.0, 0.0, 0.0);
        if strcmp(kind, 'l180')
            ed.omega.Value = omega;
        end
        e = yc.exit_errors(kind, sim);
        [dmin, pil] = yc.min_pillar_distance(kind, sim.x, sim.y);
        half_w = 0.5 * ed.width.Value;
        [sug_pre, sug_post] = suggest_adj(K, C, pre_adj, post_adj);
        v = ed.v.Value;
        lat_acc = v * deg2rad(omega) / 1000.0;  % [m/s²]

        paste_text = sprintf('#define %-26s%.1ff\n#define %-26s%.1ff\n', kd.adj{1}, sug_pre, kd.adj{2}, sug_post);
        side = lr(is_right());
        hit = '';
        if dmin - half_w < 0, hit = '  ← 当たる'; end
        lines = {
            sprintf('%s (%s)', kd.label, side)
            sprintf('v %.0f mm/s  ω %.1f dps  α %.0f dps²', v, omega, ed.alpha.Value)
            sprintf('  横の加速度の最大 %.2f m/s² (%.2f G)', lat_acc, lat_acc / 9.81)
            sprintf('機体と同じ前後のオフセット: pre %.2f  post %.2f mm', pre0, post0)
            sprintf('  + ADJ で実際に走る:       pre %.2f  post %.2f mm', pre0 + pre_adj, post0 + post_adj)
            sprintf('曲がる時間 %.0f ms、曲がる間の道のり %.1f mm', shape(5) * 1000, shape(4))
            sprintf('スリップ K=%g C=%g: β の最大 %.2f°', K, C, max(abs(sim.beta)))
            ''
            '出口のずれ (後ろのオフセットの後):'
            sprintf('  前後 %+.2f mm (+ で行き過ぎ)', e.along)
            sprintf('  横   %+.2f mm (+ で外)  → 延長後 %+.2f mm', e.lat_post, e.lat_final)
            sprintf('  進む向き %+.2f° (機体の向きは指令どおり)', e.travel_err)
            sprintf('柱との距離 (中心の軌道): %.1f mm (柱 %.0f,%.0f)', dmin, pil(1), pil(2))
            sprintf('  − 幅/2 = %.1f mm%s', dmin - half_w, hit)
            '  (機体を幅の円とみた目安。角の出っ張りは見ていない)'
            ''
            'ずれを消す調整分 (今の K, C で):'
            };
        lines = [lines; splitlines(strtrim(paste_text))];
        if strcmp(kind, 'l180')
            lines(end+1) = {sprintf('  横 %+.2f mm は前後の直進では直せない', e.lat_final)};
            lines(end+1) = {'  (機体は ω をスリップなしで求めている)'};
        end
        if isfield(fit_texts, kind)
            lines = [lines; {''}; splitlines(fit_texts.(kind))];
        end
        result.Value = lines;
        draw(kind, sim, ideal, half_w);
    end

    % ---- 描く ----
    function draw(kind, sim, ideal, half_w)
        kd = kinds.(kind);
        if is_right(), mir = -1.0; else, mir = 1.0; end  % 計算は左に曲がる形、右なら x を反転
        ax = ax_map;
        cla(ax);
        hold(ax, 'on');
        xs = sim.x * mir; ys = sim.y;
        all_x = [xs; kd.start(1) * mir];
        all_y = [ys; kd.start(2)];
        klogs = logs_of(kind);
        for lg = klogs
            all_x = [all_x; lg.x * mir]; %#ok<AGROW>
            all_y = [all_y; lg.y]; %#ok<AGROW>
        end

        % 区画の線と柱
        pil = yc.pillars_near(kind, all_x, all_y, 60.0);
        H = 6.0;
        xline(ax, unique(pil(:, 1)), 'Color', [0.85 0.85 0.85], 'LineWidth', 0.8);
        yline(ax, unique(pil(:, 2)), 'Color', [0.85 0.85 0.85], 'LineWidth', 0.8);
        px = pil(:, 1)'; py = pil(:, 2)';
        patch(ax, [px - H; px + H; px + H; px - H], [py - H; py - H; py + H; py + H], [0.2 0.2 0.2], ...
            'EdgeColor', 'none');

        % 機体の幅(向きは機体の向き θ)
        th = deg2rad(sim.theta);
        for sgn = [1, -1]
            plot(ax, (sim.x - sgn * half_w * sin(th)) * mir, sim.y + sgn * half_w * cos(th), ...
                'Color', [0.6 0.75 0.9], 'LineWidth', 0.6);
        end

        hs = gobjects(0); names = {};
        hs(end+1) = plot(ax, ideal.x * mir, ideal.y, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0);
        names{end+1} = 'スリップなし';
        colors = {[0.17 0.63 0.17], [0.84 0.15 0.16], [0.17 0.63 0.17], [0.74 0.74 0.13]};
        for ph = 0:3
            idx = find(sim.phase == ph);
            if isempty(idx), continue; end
            if idx(1) > 1, idx = [idx(1) - 1; idx]; end  %#ok<AGROW> % 前の区間とつなげる
            h = plot(ax, xs(idx), ys(idx), 'Color', colors{ph + 1}, 'LineWidth', 2.0);
            if ph == 1
                hs(end+1) = h; %#ok<AGROW>
                names{end+1} = '旋回 (緑: 前後のオフセット)'; %#ok<AGROW>
            end
        end
        hs(end+1) = plot(ax, kd.exit(1) * mir, kd.exit(2), 'k+', 'MarkerSize', 12, 'LineWidth', 2);
        names{end+1} = '出口の目標';

        slope = slope_ed.Value;
        for lg = klogs
            h = plot(ax, lg.x * mir, lg.y, 'LineWidth', 1.2);
            hs(end+1) = h; %#ok<AGROW>
            names{end+1} = sprintf('%s (車輪+ジャイロ)', lg.name); %#ok<AGROW>
            if slope > 0
                d = lg.sensor_lateral(lg.win, slope) - lg.kin_lateral(lg.win);
                plot(ax, (lg.x(lg.win) + d * lg.n_out(1)) * mir, lg.y(lg.win) + d * lg.n_out(2), '.', ...
                    'MarkerSize', 6, 'Color', h.Color);
            end
        end
        axis(ax, 'equal');
        xlim(ax, [min([all_x; pil(:, 1)]) - 20, max([all_x; pil(:, 1)]) + 20]);
        ylim(ax, [min([all_y; pil(:, 2)]) - 20, max([all_y; pil(:, 2)]) + 20]);
        title(ax, '軌道 (点: ログの壁センサーで見た位置)');
        legend(ax, hs, names, 'Location', 'best', 'FontSize', 8, 'Interpreter', 'none');

        % 曲がった後の横のずれ
        ax2 = ax_lat;
        cla(ax2);
        hold(ax2, 'on');
        ex = kd.exit(1); ey = kd.exit(2);
        ux = kd.exit_dir(1); uy = kd.exit_dir(2);
        nx = uy; ny = -ux;
        m = sim.phase >= 2;
        along = (sim.x(m) - ex) * ux + (sim.y(m) - ey) * uy;
        lat = (sim.x(m) - ex) * nx + (sim.y(m) - ey) * ny;
        hs = plot(ax2, along, lat, 'Color', [0.84 0.15 0.16]);
        names = {'シミュレーション'};
        K = ed.K.Value; C = ed.C.Value;
        b = 0.0;
        if isfield(fit_b, kind), b = fit_b.(kind); end
        for lg = klogs
            a = lg.along_after_turn(lg.win);
            l_k = lg.kin_lateral(lg.win);
            h = plot(ax2, a, l_k, '--', 'LineWidth', 0.8);
            if slope > 0
                l_s = lg.sensor_lateral(lg.win, slope);
                hs(end+1) = plot(ax2, a, l_s, '.', 'MarkerSize', 6, 'Color', h.Color); %#ok<AGROW>
                names{end+1} = lg.name; %#ok<AGROW>
                model = lg.slip_model(K, C)' + b * (1.0 - 2.0 * lg.right);
                plot(ax2, a, l_k + model, 'LineWidth', 1.0, 'Color', h.Color);
            end
        end
        yline(ax2, 0.0, 'Color', [0.6 0.6 0.6]);
        xlabel(ax2, '出口の目標からの前後 [mm]');
        ylabel(ax2, '横 [mm] (+ が外)');
        title(ax2, '曲がった後の横のずれ  点: 壁センサー、破線: 車輪+ジャイロ、実線: 車輪+ジャイロ+スリップ(今の K, C、合わせた b)', ...
            'FontSize', 9, 'FontWeight', 'normal');
        legend(ax2, hs, names, 'Location', 'best', 'FontSize', 8, 'NumColumns', 2, 'Interpreter', 'none');
        grid(ax2, 'on');
    end

    function r = is_right()
        r = strcmp(dir_dd.Value, 'R');
    end

    function out = logs_of(kind)
        out = logs(arrayfun(@(lg) strcmp(lg.kind, kind), logs));
        out = reshape(out, 1, []);
    end

    % ---- ログ ----
    function open_logs()
        start = fullfile(log_dir, 'slalom');
        if ~isfolder(start), start = log_dir; end
        [f, d] = uigetfile({'*.csv;*.bin', 'log (*.csv, *.bin)'; '*.*', 'all'}, 'SLALOM テストのログ', ...
            [start filesep], 'MultiSelect', 'on');
        figure(fig);
        if isequal(f, 0), return; end
        add_logs(fullfile(d, cellstr(f)));
    end

    function add_logs(paths)
        for k = 1:numel(paths)
            p = paths{k};
            if ~isfile(p) && isfile(fullfile(yc.root(), p))
                p = fullfile(yc.root(), p);  % リポジトリの一番上から見たパスでもよい
            end
            try
                lg = yc.TurnLog(p, params);
            catch ex
                uialert(fig, sprintf('%s: %s', p, ex.message), 'ログ', 'Icon', 'warning');
                continue
            end
            logs(end+1) = lg; %#ok<AGROW>
        end
        refresh_list();
        if ~isempty(logs)
            last = logs(end);
            if last.right, dir_dd.Value = 'R'; else, dir_dd.Value = 'L'; end
            if ~strcmp(kind_dd.Value, last.kind)
                kind_dd.Value = last.kind;
                load_from_params();
                return
            end
        end
        update();
    end

    function refresh_list()
        items = arrayfun(@(lg) sprintf('%s: %s %s ω %.0f dps', lg.name, lg.kind, lr(lg.right), ...
            lg.peak_omega), logs, 'UniformOutput', false);
        log_list.Items = items;
        log_list.ItemsData = 1:numel(logs);
        log_list.Value = {};
    end

    function remove_log()
        sel = log_list.Value;
        if iscell(sel), sel = cell2mat(sel); end
        logs(sel) = [];
        refresh_list();
        update();
    end

    function clear_logs()
        logs = yc.TurnLog.empty;
        fit_texts = struct();
        fit_b = struct();
        refresh_list();
        update();
    end

    function estimate_slope()
        start = fullfile(log_dir, 'search');
        if ~isfolder(start), start = log_dir; end
        [f, d] = uigetfile({'*.csv;*.bin', 'log (*.csv, *.bin)'; '*.*', 'all'}, ...
            '探索のログ (壁の制御をしながら直進している所を使う)', [start filesep]);
        figure(fig);
        if isequal(f, 0), return; end
        th_l = 180.0; th_r = 150.0;
        if isfield(params, 'WALL_TH_L'), th_l = params.WALL_TH_L; end
        if isfield(params, 'WALL_TH_R'), th_r = params.WALL_TH_R; end
        [k, n, found] = yc.estimate_side_slope(yc.load_log(fullfile(d, f)), th_l, th_r);
        if isnan(k)
            uialert(fig, '使える直進が見つからなかった', '傾き', 'Icon', 'info');
            return
        end
        detail = strjoin(arrayfun(@(i) sprintf('  %7.2fs %s %+6.1f (r=%+.2f)', found.t(i), found.side(i), ...
            found.slope(i), found.r(i)), 1:height(found), 'UniformOutput', false), newline);
        uialert(fig, sprintf('中央値 %.1f AD/mm (%d 個)\n%s\n\n壁に近いほど値の変わり方は大きくなるので、だいたいの値。', ...
            k, n, detail), '傾き', 'Icon', 'info');
        slope_ed.Value = round(k, 1);
        update();
    end

    function fit()
        kind = kind_dd.Value;
        klogs = logs_of(kind);
        if isempty(klogs)
            uialert(fig, sprintf('%s のログがない', kind), '合わせる', 'Icon', 'info');
            return
        end
        d = uiprogressdlg(fig, 'Title', '合わせる', 'Message', 'K と C を探している…', 'Indeterminate', 'on');
        try
            res = yc.fit_slip(klogs, slope_ed.Value, bias_cb.Value, refine_cb.Value);
        catch ex
            close(d);
            uialert(fig, ex.message, '合わせる', 'Icon', 'info');
            return
        end
        close(d);
        fit_b.(kind) = res.b;
        ed.K.Value = res.K;
        ed.C.Value = res.C;
        fit_texts.(kind) = res.text;
        update();
    end
end


function s = lr(right)
if right
    s = '右';
else
    s = '左';
end
end
