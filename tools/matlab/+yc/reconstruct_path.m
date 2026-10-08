function [xs, ys] = reconstruct_path(L, x0, y0, heading0_deg, i0, i1)
%RECONSTRUCT_PATH ログの dist と angle から、i0〜i1 の行の軌道を作る(x 右・y 上)。
%   heading0_deg は i0 の行の向き。dist が急に減った所(制御を有効にし直して 0 に戻った所)は
%   進んでいないものとする。車輪とジャイロだけで作るので、横滑りは見えない。
if nargin < 2 || isempty(x0), x0 = 0; end
if nargin < 3 || isempty(y0), y0 = 0; end
if nargin < 4 || isempty(heading0_deg), heading0_deg = 90; end
if nargin < 5 || isempty(i0), i0 = 1; end
if nargin < 6 || isempty(i1), i1 = numel(L.dist); end
d = L.dist(i0:i1);
a = L.angle(i0:i1);
ds = diff(d(:));
ds(ds < -5.0) = 0.0;
th = heading0_deg + (a(2:end) - a(1));
xs = x0 + [0; cumsum(ds .* cosd(th(:)))];
ys = y0 + [0; cumsum(ds .* sind(th(:)))];
end
