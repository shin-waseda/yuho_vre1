function make_markers(out_dir)
%MAKE_MARKERS 真上から撮って機体の位置と向きを測るための ArUco マーカーの STL を作る(2026-11-17)。
%   make_markers()            % tools/camera/stl に書く
%   make_markers(out_dir)
%
%   どれも白い樹脂で刷り、凸(黒にするマス)の上面をペンで塗る。模様は DICT_4X4_50(4×4 のマスと黒い縁)。
%   - marker_robot_id0.stl: 機体のマーカー(板だけ。台はユーザーが作り、その上に載せる)
%   - ref_id1_SW.stl など: 迷路の四隅の柱に載せる基準のマーカー(4×4 なら柱の中心が (0,0)・(720,0)・(720,720)・(0,720) mm)。
%     柱の外側の 2 面に当てるつめが付いていて、正しい角に載せると、どれも模様の上が北になる。
%   作った後、上から見た形を画像にして readArucoMarker で読み、ID と向きが合っているかを確かめる(鏡写しの間違いを防ぐ)。
%
%   寸法は下の P(mm)。マーカー 30mm 角(1 マス 5mm)の外に白い余白 4mm を付ける(読み取りには周りに白い縁が要る)。
if nargin < 1 || isempty(out_dir)
    out_dir = fullfile(fileparts(mfilename('fullpath')), 'stl');
end
if ~isfolder(out_dir), mkdir(out_dir); end

P.family = "DICT_4X4_50";
P.cell = 5.0;          % 1 マス[mm](黒い縁を入れて 6 マス = 30mm)
P.margin = 4.0;        % 黒い縁の外の白い余白[mm]
P.relief = 0.6;        % 凸の高さ[mm](高いと斜めから撮ったときに側面が写って角がぼける)
P.base_robot = 1.2;    % 機体のマーカーの板の厚さ[mm]
P.base_ref = 2.0;      % 基準のマーカーの板の厚さ[mm]
P.post = 12.0;         % 柱の太さ[mm]
P.post_gap = 0.15;     % つめと柱のすき間[mm]
P.claw_t = 2.0;        % つめの厚さ[mm]
P.claw_depth = 8.0;    % つめが柱の上から下りる長さ[mm]

% 機体のマーカー
B0 = marker_bits(P.family, 0);
[V, F] = marker_tile(B0, P, P.base_robot);
write_stl(fullfile(out_dir, 'marker_robot_id0.stl'), V, F);
check_top_view(V, F, P, 0);

% 基準のマーカー(柱の角ごとに、つめを付ける外側の 2 面: −x 西、+x 東、−y 南、+y 北)
refs = {1, 'SW', [-1 -1]; 2, 'SE', [1 -1]; 3, 'NE', [1 1]; 4, 'NW', [-1 1]};
for k = 1:size(refs, 1)
    id = refs{k, 1};
    B = marker_bits(P.family, id);
    [V, F] = marker_tile(B, P, P.base_ref);
    s = refs{k, 3};
    h = P.post / 2 + P.post_gap;          % 柱の中心からつめの内側の面まで
    L = P.post / 2;                        % つめの長さの半分(柱の幅だけ)
    % x の外側のつめ(西か東)
    x0 = s(1) * h; x1 = s(1) * (h + P.claw_t);
    [V, F] = add_box(V, F, [min(x0, x1) max(x0, x1)], [-L L], [-P.claw_depth 0]);
    % y の外側のつめ(南か北)。角で x のつめとつながるように、長さを厚さの分だけ伸ばす
    y0 = s(2) * h; y1 = s(2) * (h + P.claw_t);
    xs = sort([-L * s(1), s(1) * (h + P.claw_t)]);
    [V, F] = add_box(V, F, xs, [min(y0, y1) max(y0, y1)], [-P.claw_depth 0]);
    write_stl(fullfile(out_dir, sprintf('ref_id%d_%s.stl', id, refs{k, 2})), V, F);
    check_top_view(V, F, P, id);
end

% 測るときに使う寸法を残す(写真から位置を出すスクリプトが読む)
M.family = P.family;
M.size = 6 * P.cell;
M.relief = P.relief;
M.base_robot = P.base_robot;
M.base_ref = P.base_ref;
save(fullfile(out_dir, 'markers.mat'), 'M');
fprintf('written to %s\n', out_dir);
end


function B = marker_bits(family, id)
% 6×6 のマス(黒い縁を含む)。true が黒。行 1 が上(北)、列 1 が左(西)
I = generateArucoMarker(family, id, 60);
B = I(5:10:end, 5:10:end) < 128;
end


function [V, F] = marker_tile(B, P, base)
% 板(白)と、黒いマスの凸。板の真ん中が原点、上面(白)が z = 0 で、凸は z = 0〜relief、板は z = −base〜0
S = size(B, 1) * P.cell;
half = S / 2 + P.margin;
V = zeros(0, 3); F = zeros(0, 3);
[V, F] = add_box(V, F, [-half half], [-half half], [-base 0]);
for r = 1:size(B, 1)
    for c = 1:size(B, 2)
        if ~B(r, c), continue; end
        x = -S / 2 + (c - 1) * P.cell;   % 列は西から東へ
        y = S / 2 - r * P.cell;          % 行は北から南へ
        [V, F] = add_box(V, F, [x x + P.cell], [y y + P.cell], [0 P.relief]);
    end
end
end


function [V, F] = add_box(V, F, xr, yr, zr)
% 直方体を足す(面は外向き)
[X, Y, Z] = ndgrid(xr, yr, zr);
v = [X(:) Y(:) Z(:)];   % 番号: 1+ (ix-1) + 2(iy-1) + 4(iz-1)
f = [1 3 2; 2 3 4;      % 下(z 小)
     5 6 7; 6 8 7;      % 上
     1 2 5; 2 6 5;      % y 小
     3 7 4; 4 7 8;      % y 大
     1 5 3; 3 5 7;      % x 小
     2 4 6; 4 8 6];     % x 大
F = [F; f + size(V, 1)];
V = [V; v];
end


function write_stl(path, V, F)
stlwrite(triangulation(F, V), path);
end


function check_top_view(V, F, P, id)
% 上から見た形を画像にして(凸の上面を黒)、readArucoMarker で読む。ID と、1 番目の角が左上(北西)であることを確かめる
px = 0.25;  % [mm/画素]
lim = max(abs(V(:, 1:2)), [], 'all') + 5;
n = round(2 * lim / px);
img = 255 * ones(n, n, 'uint8');
top = V(:, 3) > P.relief - 1e-6;
for k = 1:size(F, 1)
    if ~all(top(F(k, :))), continue; end
    xy = V(F(k, :), 1:2);
    col = (xy(:, 1) + lim) / px; row = (lim - xy(:, 2)) / px;
    mask = poly2mask(col, row, n, n);
    img(mask) = 0;
end
[ids, locs] = readArucoMarker(img, P.family);
if numel(ids) ~= 1 || ids ~= id
    error('make_markers:check', 'ID %d の形を読めなかった(読めた ID: %s)', id, mat2str(ids));
end
c1 = locs(1, :, 1); ctr = mean(locs(:, :, 1), 1);
if ~(c1(1) < ctr(1) && c1(2) < ctr(2))
    error('make_markers:check', 'ID %d の向きが違う(1 番目の角が左上でない)', id);
end
fprintf('ID %d: read OK from the top view (corner 1 is north-west)\n', id);
end
