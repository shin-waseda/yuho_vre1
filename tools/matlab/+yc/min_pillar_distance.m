function [dmin, pil] = min_pillar_distance(kind, xs, ys)
%MIN_PILLAR_DISTANCE 機体の中心の軌道と柱(12mm 角)の縁との一番近い距離と、その柱 [x y]
H = 6.0;  % PILLAR_HALF_MM
P = yc.pillars_near(kind, xs, ys, 50.0);
x = xs(1:2:end); y = ys(1:2:end);
x = x(:)'; y = y(:)';
dx = max(abs(x - P(:, 1)) - H, 0.0);  % (柱の数) × (点の数)
dy = max(abs(y - P(:, 2)) - H, 0.0);
d = hypot(dx, dy);
dp = min(d, [], 2);
[dmin, k] = min(dp);
pil = P(k, :);
if isempty(dmin)
    dmin = inf; pil = [nan nan];
end
end
