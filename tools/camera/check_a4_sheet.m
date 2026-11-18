function check_a4_sheet(png_path, mat_path)
%CHECK_A4_SHEET make_a4_sheet の PDF を画像にしたものを読み、ID・黒の 1 辺・中心の位置を確かめる(2026-11-17)。
%   check_a4_sheet(png_path)             % 寸法は print/a4_sheet.mat
%   check_a4_sheet(png_path, mat_path)
%
%   画像は PDF のページ全体(A4)を描いたもの(例: PyMuPDF で 635 dpi)。画像の大きさから 1mm の画素の数を出す。
if nargin < 2 || isempty(mat_path)
    mat_path = fullfile(fileparts(mfilename('fullpath')), 'print', 'a4_sheet.mat');
end
S = load(mat_path);
sh = S.sheet;
J = imread(png_path);
ppmm = [size(J, 2) / sh.paper_mm(1), size(J, 1) / sh.paper_mm(2)];
fprintf('%d x %d px (%.3f, %.3f px/mm)\n', size(J, 2), size(J, 1), ppmm);
[ids, locs] = readArucoMarker(J, "DICT_4X4_50");
if ~isequal(sort(ids(:))', sort(sh.ids))
    error('check_a4_sheet:id', 'ID が合わない(読めたのは %s)', mat2str(ids));
end
for k = 1:numel(ids)
    L = locs(:, :, k) ./ ppmm;
    side = mean(vecnorm(L - circshift(L, -1), 2, 2));
    c = mean(L, 1); c(2) = sh.paper_mm(2) - c(2);
    j = find(sh.ids == ids(k));
    fprintf('ID %d: black %.2f mm (expected %g), center (%.2f, %.2f) expected (%.2f, %.2f)\n', ...
        ids(k), side, sh.black_mm(j), c, sh.center_paper_mm(j, :));
end
end
