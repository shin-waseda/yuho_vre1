function make_paper_markers(out_dir)
%MAKE_PAPER_MARKERS 試し撮り用の紙のマーカーの PNG を作る(2026-11-17)。
%   make_paper_markers()            % tools/camera/print に書く
%   make_paper_markers(out_dir)
%
%   - marker_robot_id10_paper.png: 機体に仮に貼るマーカー(ID 10、黒 30mm)。LED で向きを測る方法を確かめるときだけ使う
%   - cell_plate_id11-14_paper.png: 区画の床に置く基準の板(167mm 角。黒 50mm のマーカーを 2×2 で 4 枚。
%     SW 11、SE 12、NE 13、NW 14。どれも模様の上が板の北)。止まった区画の隣のマスに入れ、2 枚の壁の角に押し当てる
%   ID は yozakura(機体 0、板 5〜8)と重ならないように 10〜14 にした(同じ紙に刷っても、同じ写真に写っても取り違えない)。
%   3D プリントの STL(make_markers.m の 0〜4)とも違う番号。
%
%   PNG に解像度(635 dpi = 1mm 25 画素)を書き込んであるので、Word などに貼ると実寸で入る。印刷は 100 %(拡大・縮小なし)で。
%   刷った後、50mm の線をノギスで測って大きさを確かめる。四隅の外の短い線を結んだ四角で切る
%   (yozakura で、白い余白の中に切り取り線を引くと readArucoMarker で読めなかった)。
%
%   板の寸法: 区画の内側(180 − 柱 12 = 168mm)より 1mm 小さい 167mm 角。板の縁から黒まで 26mm 空けた。
%   機体の区画との間に壁があると、カメラから見て壁の向こう側の際の床が 50 d / (H − 50) mm 隠れる(壁 50mm、
%   カメラが壁から d 離れて高さ H。H = 400、d = 180 で約 26mm)。
if nargin < 1 || isempty(out_dir)
    out_dir = fullfile(fileparts(mfilename('fullpath')), 'print');
end
if ~isfolder(out_dir), mkdir(out_dir); end

ppmm = 25; % 1mm の画素の数(635 dpi)
family = "DICT_4X4_50";

% ---- 機体のマーカー(ID 10。黒 30mm、白い余白 4mm。3D プリントの marker_robot_id0 と同じ大きさ) ----
R.id = 10; R.black = 30; R.margin = 4;
W = 70; H = 66;
img = blank(W, H, ppmm);
T = R.black + 2 * R.margin;
x0 = (W - T) / 2; y0 = 6;
img = draw_marker(img, family, R.id, x0 + R.margin, y0 + R.margin, R.black, ppmm);
img = cut_marks(img, x0, y0, T, T, ppmm);
img = scale_bar(img, W, y0 + T + 7, ppmm);
img = label(img, W / 2, 1, sprintf('yuho robot  ArUco 4x4_50  ID %d  black %g mm', R.id, R.black), ppmm);
write_check(img, fullfile(out_dir, sprintf('marker_robot_id%d_paper.png', R.id)), family, R.id, R.black, ppmm);

% ---- 区画の床に置く基準の板(167mm 角。黒 50mm を 2×2) ----
Pl.side = 167; Pl.black = 50; Pl.gap = 15;        % マーカーの間の白[mm]
Pl.ids = [14 13; 11 12];                          % 行: 北、南 / 列: 西、東
W = 182; H = 200;
img = blank(W, H, ppmm);
x0 = (W - Pl.side) / 2; y0 = 9;
pitch = Pl.black + Pl.gap;                        % マーカーの中心の間(65mm。板の中心から ±32.5mm)
c0 = Pl.side / 2 - pitch / 2;                     % 板の左上から、左上のマーカーの中心まで
for r = 1:2
    for c = 1:2
        cx = x0 + c0 + (c - 1) * pitch; cy = y0 + c0 + (r - 1) * pitch;
        img = draw_marker(img, family, Pl.ids(r, c), cx - Pl.black / 2, cy - Pl.black / 2, Pl.black, ppmm);
    end
end
img = cut_marks(img, x0, y0, Pl.side, Pl.side, ppmm);
img = scale_bar(img, W, y0 + Pl.side + 6, ppmm);
img = label(img, W / 2, 1, sprintf('yuho cell plate %g mm  N up  NW %d  NE %d  SW %d  SE %d  (black %g mm)', ...
    Pl.side, Pl.ids(1, 1), Pl.ids(1, 2), Pl.ids(2, 1), Pl.ids(2, 2), Pl.black), ppmm);
write_check(img, fullfile(out_dir, sprintf('cell_plate_id%d-%d_paper.png', min(Pl.ids(:)), max(Pl.ids(:)))), ...
    family, sort(Pl.ids(:))', Pl.black, ppmm);

% 写真から位置を出すスクリプトが読む寸法(板の中心が原点、北 +y、東 +x。マーカーの中心[mm])
plate.ids = [Pl.ids(1, 1) Pl.ids(1, 2) Pl.ids(2, 1) Pl.ids(2, 2)];   % NW NE SW SE
plate.center_mm = pitch / 2 * [-1 1; 1 1; -1 -1; 1 -1];
plate.black_mm = Pl.black; plate.side_mm = Pl.side;
robot.id = R.id; robot.black_mm = R.black;
save(fullfile(out_dir, 'paper_markers.mat'), 'plate', 'robot');
end

function img = blank(W, H, ppmm)
img = 255 * ones(round(H * ppmm), round(W * ppmm), 'uint8');
end

function img = draw_marker(img, family, id, x, y, black, ppmm)
% 左上 (x, y)[mm] から黒の 1 辺 black[mm] で描く(6×6 マス)
I = generateArucoMarker(family, id, 60);
B = I(5:10:end, 5:10:end) < 128;
cell = black / 6;
px = @(mm) round(mm * ppmm);
for r = 1:6
    for c = 1:6
        if ~B(r, c), continue; end
        img(px(y + (r - 1) * cell) + 1 : px(y + r * cell), px(x + (c - 1) * cell) + 1 : px(x + c * cell)) = 0;
    end
end
end

function img = cut_marks(img, x0, y0, w, h, ppmm)
% 切り取る四角の四隅の外に、辺をのばした向きの短い線(灰色)
px = @(mm) round(mm * ppmm);
g = 120; lw = px(0.2); gap = 1.0; len = 3.0;
for xc = [x0, x0 + w]
    for yc = [y0, y0 + h]
        sx = sign(xc - (x0 + w / 2)); sy = sign(yc - (y0 + h / 2));
        ys = sort([yc + sy * gap, yc + sy * (gap + len)]);
        img(px(ys(1)) + 1 : px(ys(2)), px(xc) - (sx < 0) * lw + (1:lw)) = g;
        xs = sort([xc + sx * gap, xc + sx * (gap + len)]);
        img(px(yc) - (sy < 0) * lw + (1:lw), px(xs(1)) + 1 : px(xs(2))) = g;
    end
end
end

function img = scale_bar(img, W, yb, ppmm)
% 大きさを確かめる 50mm の線(両端に縦の線)
px = @(mm) round(mm * ppmm);
img(px(yb) + (1:px(0.3)), px((W - 50) / 2) + 1 : px((W + 50) / 2)) = 0;
for xe = [(W - 50) / 2, (W + 50) / 2 - 0.3]
    img(px(yb - 2) + 1 : px(yb + 2), px(xe) + (1:px(0.3))) = 0;
end
img = label(img, W / 2, yb + 2.5, '50 mm (print at 100 %)', ppmm);
end

function img = label(img, x, y, txt, ppmm)
rgb = insertText(repmat(img, 1, 1, 3), [round(x * ppmm), round(y * ppmm)], txt, 'FontSize', 36, ...
    'BoxOpacity', 0, 'AnchorPoint', 'CenterTop', 'TextColor', 'black');
img = rgb2gray(rgb);
end

function write_check(img, path, family, ids, black, ppmm)
imwrite(img, path, 'ResolutionUnit', 'meter', 'XResolution', ppmm * 1000, 'YResolution', ppmm * 1000);
% 読み直して確かめる: ID、黒の 1 辺の長さ、書き込んだ解像度
J = imread(path);
[got, locs] = readArucoMarker(J, family);
if ~isequal(sort(got(:))', sort(ids(:))')
    error('make_paper_markers:check', '%s: ID を読めなかった(読めたのは %s)', path, mat2str(got));
end
for k = 1:numel(got)
    L = locs(:, :, k);
    side = mean(vecnorm(L - circshift(L, -1), 2, 2)) / ppmm;
    fprintf('%s: ID %d read OK, black side %.2f mm (expected %g)\n', path, got(k), side, black);
end
info = imfinfo(path);
fprintf('  %.1f dpi, %d x %d px (%.1f x %.1f mm)\n', info.XResolution * 0.0254, info.Width, info.Height, ...
    info.Width / ppmm, info.Height / ppmm);
end
