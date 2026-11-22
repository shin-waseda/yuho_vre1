function T = analyze_slip_camera(photo_dir, type, varargin)
%ANALYZE_SLIP_CAMERA SLIP の CAMERA で撮った写真から、1 本ごとの止まった位置と向きを出す(2026-11-20)。
%   T = analyze_slip_camera('M:/.../photos/slip_turn', 'turn')
%   T = analyze_slip_camera('M:/.../photos/slip_l90', 'l90', 'From', 7, 'Skip', [12 13])
%   type は 'turn'(1 TURN: T1・T1L)/ 'l90'(2 LARGE90)/ 'l180'(3 LARGE180)。
%   写真は 1 本に 置いた所・止まった所 の 2 枚(WithStart が false なら止まった所の 1 枚)で、名前の順(iPhone の
%   IMG_xxxx)が撮った順になっていること。走りの並びは機体
%   (slip_turn_test.c の CAMERA)と同じ: 50 刻みの速さを 1 本ずつ、右を遅い順に全部 → 左を遅い順に全部。
%   1 枚ごとに camera-pose の analyze_maze に、止まる区画とだいたいの向きを渡して位置を出す。
%   置いた所の写真があれば、止まった所を置いた所に対する位置として当てはめに使える(手で置いた向きのずれを消す)。
%   結果は写真のフォルダの slip_camera.csv(fit_slip_k.py が読む。置いた所は x0_mm・y0_mm・th0_deg)。
%
%   オプション
%     From         最初の写真が何本目か(機体の FROM で選んだ番号。既定 1)
%     Skip         写真のない本の番号(走りに失敗したなど。既定 [])
%     Repeat       2 回走った(写真が 2 組ある)本の番号(FROM で同じ本から走り直したなど。既定 [])
%     WithStart    1 本に置いた所と止まった所の 2 枚か(既定 true)
%     Old3         前の並び(速さ 100 刻みで、速さごとに右 3 本 → 左 3 本。2026-11-20 の初めの TURN の 12 本)で読む(既定 false)
%     CameraPose   camera-pose のフォルダ(既定 M:/User/Desktop/school/club/camera-pose)
%     その他の名前と値は analyze_maze にそのまま渡す(Rotate、CalibFile など)
%     光った点の候補は、露出を下げた写真に合わせた値を既定にする(MinHaloSat 0、DiamMm [0.5 6]、Threshold 0.8)。
%     2026-11-20 の写真(露出を下げたもの)では、analyze_maze の既定(0.5、[2 6] から、0.95)だと前の LED は 2mm より小さく、
%     後ろの LED は明るさ 0.85 ほどで候補から落ち、止まった所の多くが LED 2 個になった
p = inputParser;
p.KeepUnmatched = true;
p.addParameter('From', 1);
p.addParameter('Skip', []);
p.addParameter('Repeat', []);
p.addParameter('WithStart', true);
p.addParameter('Old3', false);
p.addParameter('CameraPose', 'M:/User/Desktop/school/club/camera-pose');
p.parse(varargin{:});
o = p.Results;
spot = struct('MinHaloSat', 0, 'DiamMm', [0.5 6], 'Threshold', 0.8);
for f = fieldnames(p.Unmatched)'
    spot.(f{1}) = p.Unmatched.(f{1});
end
extra = namedargs2cell(spot);
addpath(o.CameraPose);

% 機体の CAMERA と同じ並び。kind は LOG_EV_SLIP_RUN の番号、cell・heading は止まる所、start は置く区画(どれも北向き)
% (迷路の座標、東 0°・北 90°)
% speeds は kCam*Speeds(old は前の kSlipSpeeds など)
switch lower(type)
    case 'turn'
        speeds = 300:50:750; old = [300 400 500 600];
        kinds = {'T1', 1, [1 1], 0, [0 0]; 'T1L', 7, [1 1], 180, [2 0]};
    case 'l90'
        speeds = 500:50:1200; old = [600 700 800 900 1000];
        kinds = {'L90R', 24, [2 2], 0, [0 0]; 'L90L', 22, [1 2], 180, [3 0]};
    case 'l180'
        speeds = 500:50:900; old = [600 700 800 900];
        kinds = {'L180R', 26, [1 1], -90, [0 0]; 'L180L', 28, [2 1], -90, [3 0]};
    otherwise
        error('analyze_slip_camera:type', 'type は turn / l90 / l180');
end
plan = struct('run', {}, 'name', {}, 'kind', {}, 'v', {}, 'cell', {}, 'heading', {}, 'start', {});
if o.Old3
    order = zeros(0, 2); % [速さ 種類]
    for i = 1:numel(old)
        order = [order; repmat([old(i) 1], 3, 1); repmat([old(i) 2], 3, 1)]; %#ok<AGROW>
    end
else
    order = [speeds(:) ones(numel(speeds), 1); speeds(:) 2 * ones(numel(speeds), 1)];
end
for r = 1:size(order, 1)
    j = order(r, 2);
    plan(r) = struct('run', r, 'name', kinds{j, 1}, 'kind', kinds{j, 2}, 'v', order(r, 1), ...
        'cell', kinds{j, 3}, 'heading', kinds{j, 4}, 'start', kinds{j, 5});
end
plan = plan([plan.run] >= o.From & ~ismember([plan.run], o.Skip));
for r = o.Repeat(:)'
    i = find([plan.run] == r, 1);
    plan = plan([1:i i:end]);
end

% 写真は JPEG だけ(フォルダに図の PNG を置いても写真に数えないように)
files = [dir(fullfile(photo_dir, '*.jpg')); dir(fullfile(photo_dir, '*.jpeg')); dir(fullfile(photo_dir, '*.JPG'))];
[~, iu] = unique(lower({files.name}));
files = files(iu); % 名前の順
per = 1 + o.WithStart; % 1 本あたりの写真の数
if numel(files) ~= per * numel(plan)
    warning('analyze_slip_camera:count', '写真 %d 枚と走り %d 本(× %d 枚)の数が合わない。少ない方まで使う(From・Skip を確かめる)', ...
        numel(files), numel(plan), per);
end
n = min(floor(numel(files) / per), numel(plan));
out_dir = fullfile(photo_dir, 'analyze');

T = table();
for i = 1:n
    pl = plan(i);
    fs = files(per * (i - 1) + (1:per));
    x0 = NaN; y0 = NaN; th0 = NaN; n0 = NaN;
    if o.WithStart
        fprintf('#%d %s v %d 置いた所: ', pl.run, pl.name, pl.v);
        p0 = analyze_maze(fullfile(fs(1).folder, fs(1).name), 'Cell', pl.start, 'HeadingDeg', 90, 'OutDir', out_dir, extra{:});
        if isempty(p0)
            continue; % analyze_maze が「できなかった」と出している
        end
        x0 = p0.x_mm; y0 = p0.y_mm; th0 = p0.th_deg; n0 = p0.n_leds_used;
    end
    fprintf('#%d %s v %d 止まった所: ', pl.run, pl.name, pl.v);
    pose = analyze_maze(fullfile(fs(end).folder, fs(end).name), 'Cell', pl.cell, 'HeadingDeg', pl.heading, ...
        'OutDir', out_dir, extra{:});
    if isempty(pose)
        continue;
    end
    row = table(pl.run, string(pl.name), pl.kind, pl.v, string(fs(end).name), pose.x_mm, pose.y_mm, pose.th_deg, ...
        pose.dth_deg, pose.n_leds_used, pose.resid_rms_mm, pose.lattice_rms_px, x0, y0, th0, n0, ...
        'VariableNames', {'run', 'name', 'kind', 'v', 'file', 'x_mm', 'y_mm', 'th_deg', 'dth_deg', 'n_leds_used', ...
        'resid_rms_mm', 'lattice_rms_px', 'x0_mm', 'y0_mm', 'th0_deg', 'n0_leds_used'});
    T = [T; row]; %#ok<AGROW>
end
if ~isempty(T)
    writetable(T, fullfile(photo_dir, 'slip_camera.csv'));
    fprintf('wrote %s (%d 本)\n', fullfile(photo_dir, 'slip_camera.csv'), height(T));
end
end
