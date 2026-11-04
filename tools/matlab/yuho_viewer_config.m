function cfg = yuho_viewer_config(root)
%YUHO_VIEWER_CONFIG 配布版のログビューア(mouse_log_viewer)を yuho のログで使うための設定。
%   cfg = yuho_viewer_config();            % root はこのリポジトリ(yc.root())
%   mouse_log_viewer('logs/search/search_0008.bin', cfg)
%
%   yuho に固有の取り決めはすべてここに書く(配布版の本体は yuho の名前を知らない)。
%   - 列の組: yc.plot_groups
%   - イベントの名前と中身: Core/Inc/app/log_event.h の LOG_EV_名前 = 番号, // a:名前 ...
%   - 迷路の大きさ: Core/Inc/params.h の MAZE_SIZE
%   - 軌道の基準: 尻当て(SETPOS)が2回終わった所 → なければ最初の STEP → なければ記録の始め
%   - 合わせ直し: STEP のたびに区画の境界(止まっていれば真ん中)へ
%   - 壁: 機体の地図(MAP_CELLS イベント)と、STEP で読んだ壁
if nargin < 1 || isempty(root)
    root = yc.root();
end
cfg = mlv.default_config();
cfg.name = 'yuho log viewer';
cfg.log_dir = fullfile(root, 'logs');
cfg.base_dir = root;  % log_viewer('logs/...') のように、リポジトリの一番上から見たパスでも開ける

[groups, default_cols, hidden] = yc.plot_groups();
cfg.groups = groups;
cfg.default_columns = default_cols;
cfg.hidden_columns = hidden;

header = fullfile(root, 'Core', 'Inc', 'app', 'log_event.h');
cfg.events.table = @() mlv.event_table_from_c(header, 'LOG_EV_');  % 開くたびに読む(書き換わっていれば読み直す)
cfg.events.exclude = 'STEP_INFO';

cfg.maze.kind = 'classic';
try
    p = yc.read_params(fullfile(root, 'Core', 'Inc', 'params.h'));
    if isfield(p, 'MAZE_SIZE'), cfg.maze.size = p.MAZE_SIZE; end
catch
    % params.h が読めなければ、クラシックの 16
end

cfg.trajectory.anchor = @anchor;
cfg.trajectory.reanchor = @reanchor;
cfg.trajectory.reanchor_label = 'STEP で合わせ直す';

map = struct();
map.name = '機体の地図';
map.fn = @(L, c) map_walls(L, c, header);
map.styles = struct('kind', {"known", "unknown"}, ...
    'color', {[0.25 0.25 0.25], [0.6 0.6 0.6]}, 'width', {3, 0.8}, 'style', {'-', ':'});
map.dedup = false;          % 隣の区画と同じ壁が2回出るが、どちらも描く(今までと同じ)
map.expand_region = false;  % 図の範囲は軌道と STEP で決め、その中の地図だけ描く

steps = struct();
steps.name = '読んだ壁';
steps.fn = @step_walls;
steps.styles = mlv.walls.default_styles();  % ある = 赤、ない = 緑の破線

cfg.wall_layers = {map, steps};  % 前の層が下になる
end


% =====================================================================
function a = anchor(L, cfg)
% スタート区画の真ん中・北向きとする行
S = cfg.maze.cell_mm;
a = struct('row', 1, 'x', 0.5 * S, 'y', 0.5 * S, 'heading', 90.0, 'name', '記録の始め');
[idx, ~, txt] = mlv.find_events(L, 'SETPOS', cfg);
ends = [];
for k = 1:numel(idx)
    v = mlv.event_values(txt(k));
    if isfield(v, 'phase') && v.phase >= 0.5
        ends(end+1) = idx(k); %#ok<AGROW>
    end
end
if numel(ends) >= 2
    a.row = ends(2); a.name = '尻当て2回目の終わり';
    return
end
idx = mlv.find_events(L, 'STEP', cfg);
if ~isempty(idx)
    a.row = idx(1); a.name = '最初の STEP';
end
end


function A = reanchor(L, cfg, ~)
% STEP のたびに、位置と向きを区画の境界(止まっていれば真ん中)に合わせ直す: [行, x, y, 向き(deg)]
S = cfg.maze.cell_mm;
[sidx, ~, stxt] = mlv.find_events(L, 'STEP', cfg);
A = zeros(0, 4);
for k = 1:numel(sidx)
    i = sidx(k);
    v = mlv.event_values(stxt(k));
    if ~all(isfield(v, {'x', 'y', 'heading'})), continue; end
    h = round(v.heading);
    cx = (v.x + 0.5) * S; cy = (v.y + 0.5) * S;
    if isfield(L, 'target') && abs(L.target(i)) > 10.0
        % 境界で壁を読んだ: 区画の真ん中から、来た向きへ半区画戻った所
        hd = mlv.heading_deg(h);
        cx = cx - round(cosd(hd)) * 0.5 * S;
        cy = cy - round(sind(hd)) * 0.5 * S;
    end
    A(end+1, :) = [i, cx, cy, mlv.heading_deg(h)]; %#ok<AGROW>
end
end


function w = step_walls(L, cfg)
% STEP で読んだ壁(前・右・左。bit0 前, bit1 右, bit2 左)
S = cfg.maze.cell_mm;
[sidx, ~, stxt] = mlv.find_events(L, 'STEP', cfg);
w = struct('row', zeros(0, 1), 'seg', zeros(0, 4), 'kind', strings(0, 1));
bits = [1, 2, 4]; turns = [0, 1, -1];
for k = 1:numel(sidx)
    v = mlv.event_values(stxt(k));
    if ~all(isfield(v, {'x', 'y', 'heading', 'walls'})), continue; end
    cx = round(v.x); cy = round(v.y); h = round(v.heading); wb = round(v.walls);
    for b = 1:3
        w.row(end+1, 1) = sidx(k);
        w.seg(end+1, :) = mlv.wall_segment(cx, cy, mod(h + turns(b), 4), S);
        if bitand(wb, bits(b)) ~= 0
            w.kind(end+1, 1) = "wall";
        else
            w.kind(end+1, 1) = "open";
        end
    end
end
end


function w = map_walls(L, cfg, header)
% MAP_CELLS イベントから機体の地図(1区画1バイト)を作り、壁にする。
% 同じ区画が何度も出たら、後のもの(走りの終わりに近いもの)を使う。
w = struct('row', zeros(0, 1), 'seg', zeros(0, 4), 'kind', strings(0, 1));
if ~all(isfield(L, {'ev', 'ev_a', 'ev_b', 'ev_c', 'ev_d', 'ev_e'})), return; end
code = 45;  % LOG_EV_MAP_CELLS(log_event.h にあればそちらを使う)
tbl = mlv.event_table_from_c(header, 'LOG_EV_');
for key = cell2mat(keys(tbl))
    ent = tbl(key);
    if strcmp(ent{1}, 'MAP_CELLS'), code = key; end
end
rows = find(round(L.ev) == code);
if isempty(rows), return; end
N = cfg.maze.size;
S = cfg.maze.cell_mm;
cells = nan(N, N);  % cells(x+1, y+1)
wcols = {'ev_b', 'ev_c', 'ev_d', 'ev_e'};
for r = rows'
    index = round(L.ev_a(r));
    for k = 1:4
        v = round(L.(wcols{k})(r));
        bytes = [bitand(v, 255), bitand(bitshift(v, -8), 255)];
        for j = 1:2
            c = index + 2 * (k - 1) + (j - 1);
            if c >= 0 && c < N * N
                cells(mod(c, N) + 1, floor(c / N) + 1) = bytes(j);
            end
        end
    end
end
% 北0x8 東0x4 南0x2 西0x1。下位4bit 探索用(未知は「ない」)、上位4bit 最短用(未知は「ある」)
dbits = [8, 4, 2, 1];
for cx = 0:N - 1
    for cy = 0:N - 1
        byte = cells(cx + 1, cy + 1);
        if isnan(byte), continue; end
        for d = 0:3
            lo = bitand(byte, dbits(d + 1)) ~= 0;
            hi = bitand(byte, dbits(d + 1) * 16) ~= 0;
            if lo && hi
                kind = "known";
            elseif hi && ~lo
                kind = "unknown";
            else
                continue
            end
            w.row(end+1, 1) = 0;
            w.seg(end+1, :) = mlv.wall_segment(cx, cy, d, S);
            w.kind(end+1, 1) = kind;
        end
    end
end
end
