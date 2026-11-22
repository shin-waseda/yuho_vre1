function make_a4_sheet(out_dir)
%MAKE_A4_SHEET 機体を真ん中に置いて、前後左右のマーカーと一緒に撮る A4 の紙(PDF)を作る(2026-11-17)。
%   make_a4_sheet()            % tools/camera/print に書く
%   make_a4_sheet(out_dir)
%
%   試し撮り用(ユーザーの案)。A4 縦の真ん中に yuho を置く枠を描き、周りに ArUco(DICT_4X4_50)を 8 枚置く:
%   四隅と前後の真ん中に黒 50mm(ID 15〜20)、左右の真ん中に黒 40mm(ID 21、22)。ID は yozakura(0、5〜8)・
%   床の板(11〜14)・機体(10)と重ならない。機体は紙の上(マーカーと同じ面)に置くので、LED の高さは紙からの高さになる。
%   機体の外形は docs/robot_geometry.md の 4 輪の真ん中から 前 62.34、後ろ 37.66、左右 35mm。枠は外形の 1mm 外側。
%
%   PDF は MATLAB の図を通さずに直接書く(図の print では、画面の拡大 125 % が乗って 1.25 倍になり、紙からはみ出した。
%   マスを 1 つずつ塗るとマスの間に細い線も出た)。1 枚のマーカーの黒いマスは 1 つの道(path)にまとめて塗る。
%   刷った後、100 % で印刷したか 50mm の線をノギスで確かめる。PDF を画像にして確かめるのは check_a4_sheet。
if nargin < 1 || isempty(out_dir)
    out_dir = fullfile(fileparts(mfilename('fullpath')), 'print');
end
if ~isfolder(out_dir), mkdir(out_dir); end

family = "DICT_4X4_50";
PW = 210; PH = 297;                 % A4 縦[mm]
front = 62.34; back = 37.66; half_w = 35.0;
% 紙の座標(左下が原点、上 +y)。機体の外形の真ん中を紙の真ん中に置く
ox = PW / 2; oy = PH / 2 - (front - back) / 2;   % 4 輪の真ん中(機体の原点)の紙での位置

% マーカー: ID、中心(紙の座標)、黒の 1 辺。印刷できない縁(約 5mm)と白い余白を避けて、紙の端から黒まで 12mm
big = 50; small = 40; edge = 12;
cxL = edge + big / 2; cxR = PW - edge - big / 2;
cyB = edge + big / 2 + 8; cyT = PH - edge - big / 2 - 8;   % 上下は文字と 50mm の線の分をあける
M = [ ...
    15, cxL,     cyT,     big;   % 左上
    16, PW / 2,  cyT,     big;   % 前
    17, cxR,     cyT,     big;   % 右上
    18, cxL,     cyB,     big;   % 左下
    19, PW / 2,  cyB,     big;   % 後ろ
    20, cxR,     cyB,     big;   % 右下
    21, edge + small / 2,      PH / 2,  small;   % 左
    22, PW - edge - small / 2, PH / 2,  small];  % 右

% ---- PDF の中身(単位は mm で書き、先頭の cm で pt に直す) ----
s = {};
s{end + 1} = sprintf('%.6f 0 0 %.6f 0 0 cm', 72 / 25.4, 72 / 25.4);
s{end + 1} = '0 g';
for k = 1:size(M, 1)
    s{end + 1} = marker_path(family, M(k, 1), M(k, 2), M(k, 3), M(k, 4)); %#ok<AGROW>
end
gray = 0.55;
s{end + 1} = sprintf('%.2f G 0.18 w', gray);
rx = [ox - half_w - 1, ox + half_w + 1]; ry = [oy - back - 1, oy + front + 1];
s{end + 1} = sprintf('%.3f %.3f %.3f %.3f re S', rx(1), ry(1), diff(rx), diff(ry));          % 機体を置く枠
s{end + 1} = line_cmd(ox + [-4 4], [oy oy]);                                                   % 4 輪の真ん中の十字
s{end + 1} = line_cmd([ox ox], oy + [-4 4]);
ya = oy + front - 12;                                                                           % 前を示す矢印
s{end + 1} = line_cmd([ox ox], [ya - 8 ya]);
s{end + 1} = sprintf('%.3f %.3f m %.3f %.3f l %.3f %.3f l S', ox - 2.5, ya - 3, ox, ya, ox + 2.5, ya - 3);
s{end + 1} = '0 G 0.3 w';                                                                       % 50mm の線
yb = 9;
s{end + 1} = line_cmd(PW / 2 + [-25 25], [yb yb]);
s{end + 1} = line_cmd(PW / 2 - [25 25], yb + [-2 2]);
s{end + 1} = line_cmd(PW / 2 + [25 25], yb + [-2 2]);
% 文字(Helvetica。真ん中そろえは文字の幅を 1 文字 0.55 × 大きさで見積もる)
s{end + 1} = text_cmd(PW / 2, PH - 7, 3.0, 0, 'yuho LED test sheet - print at 100 %, NOT "fit to page" - ArUco 4x4_50 ID 15-22');
s{end + 1} = text_cmd(PW / 2, yb - 4, 2.5, 0, '50 mm');
s{end + 1} = text_cmd(ox, oy + 8, 3.0, gray, 'yuho');
for k = 1:size(M, 1)
    s{end + 1} = text_cmd(M(k, 2), M(k, 3) - M(k, 4) / 2 - 3.5, 2.5, 0.4, sprintf('ID %d', M(k, 1))); %#ok<AGROW>
end
content = strjoin(s, newline);

pdf_path = fullfile(out_dir, 'yuho_led_test_sheet_A4.pdf');
write_pdf(pdf_path, content, PW * 72 / 25.4, PH * 72 / 25.4);

% 写真から位置を出すスクリプトが読む寸法(機体の原点(4 輪の真ん中)を原点、前 +x、左 +y[mm])
sheet.ids = M(:, 1)';
sheet.center_body_mm = [M(:, 3) - oy, -(M(:, 2) - ox)];   % 紙の上 = 機体の前
sheet.black_mm = M(:, 4)';
sheet.center_paper_mm = M(:, 2:3);                         % 紙の左下が原点、上 +y(確かめに使う)
sheet.robot_origin_on_paper_mm = [ox oy];
sheet.paper_mm = [PW PH];
save(fullfile(out_dir, 'a4_sheet.mat'), 'sheet');
fprintf('wrote %s\n', pdf_path);
end

function p = marker_path(family, id, cx, cy, black)
% 黒いマスを四角の道にして、まとめて 1 回で塗る(マスの間に線が出ない)
I = generateArucoMarker(family, id, 60);
B = I(5:10:end, 5:10:end) < 128;
c = black / 6;
x0 = cx - black / 2; ytop = cy + black / 2;
parts = {};
for r = 1:6
    for k = 1:6
        if ~B(r, k), continue; end
        parts{end + 1} = sprintf('%.4f %.4f %.4f %.4f re', x0 + (k - 1) * c, ytop - r * c, c, c); %#ok<AGROW>
    end
end
p = [strjoin(parts, ' ') ' f'];
end

function t = line_cmd(x, y)
t = sprintf('%.3f %.3f m %.3f %.3f l S', x(1), y(1), x(2), y(2));
end

function t = text_cmd(xc, y, size_mm, gray, str)
w = 0.55 * size_mm * strlength(str);
str = strrep(strrep(strrep(str, '\', '\\'), '(', '\('), ')', '\)');
t = sprintf('BT %.2f g /F1 %.3f Tf %.3f %.3f Td (%s) Tj ET', gray, size_mm, xc - w / 2, y, str);
end

function write_pdf(path, content, wpt, hpt)
% 1 ページの PDF を書く(フォントは標準の Helvetica)
objs = { ...
    '<< /Type /Catalog /Pages 2 0 R >>', ...
    '<< /Type /Pages /Kids [3 0 R] /Count 1 >>', ...
    sprintf('<< /Type /Page /Parent 2 0 R /MediaBox [0 0 %.4f %.4f] /Contents 4 0 R /Resources << /Font << /F1 5 0 R >> >> >>', wpt, hpt), ...
    sprintf('<< /Length %d >>\nstream\n%s\nendstream', numel(unicode2native(content, 'US-ASCII')), content), ...
    '<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>'};
out = sprintf('%%PDF-1.4\n');
offs = zeros(1, numel(objs));
for k = 1:numel(objs)
    offs(k) = numel(out);
    out = [out sprintf('%d 0 obj\n%s\nendobj\n', k, objs{k})]; %#ok<AGROW>
end
xref = numel(out);
out = [out sprintf('xref\n0 %d\n0000000000 65535 f \n', numel(objs) + 1)];
for k = 1:numel(objs)
    out = [out sprintf('%010d 00000 n \n', offs(k))]; %#ok<AGROW>
end
out = [out sprintf('trailer\n<< /Size %d /Root 1 0 R >>\nstartxref\n%d\n%%%%EOF\n', numel(objs) + 1, xref)];
fid = fopen(path, 'w');
fwrite(fid, out, 'char');
fclose(fid);
end
