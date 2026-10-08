function P = pillars_near(kind, xs, ys, margin)
%PILLARS_NEAR 軌道の近くの柱の中心(N×2)
if nargin < 4, margin = 200.0; end
S = 180.0;
y0 = yc.turn_kinds().(kind).pillar_y0;
i0 = floor((min(xs) - margin - 90.0) / S);
i1 = ceil((max(xs) + margin - 90.0) / S);
j0 = floor((min(ys) - margin - y0) / S);
j1 = ceil((max(ys) + margin - y0) / S);
[J, I] = ndgrid(j0:j1, i0:i1);  % yuho_common と同じ順(i が外、j が内)
P = [90.0 + S * I(:), y0 + S * J(:)];
end
