function [k, n, found] = estimate_side_slope(L, th_l, th_r, min_range_mm, min_r)
%ESTIMATE_SIDE_SLOPE 横の壁センサー(L, R)の傾き [AD/mm] を探索のログから見積もる。
%   直進(omega_ref = 0、target > 200)で、車輪とジャイロから求めた横の動きと L / R の値の回帰の傾きを取る。
%   左へ動くと L は増え R は減るはず(その向きになったものだけ使う)。
%   k: 傾きの |値| の中央値(なければ NaN)、n: 使った数、found: table(時刻, 側, 傾き, r)
if nargin < 2, th_l = 180.0; end
if nargin < 3, th_r = 150.0; end
if nargin < 4, min_range_mm = 3.0; end
if nargin < 5, min_r = 0.6; end
k = NaN; n = 0;
found = table('Size', [0 4], 'VariableTypes', {'double', 'string', 'double', 'double'}, ...
    'VariableNames', {'t', 'side', 'slope', 'r'});
need = {'omega_ref', 'target', 'angle', 'angle_ref', 'wall_ofs', 'dist', 'ad_l', 'ad_r'};
if ~all(isfield(L, need))
    return
end
idx = find(abs(L.omega_ref) < 1.0 & L.target > 200.0);
if isempty(idx)
    return
end
brk = [0; find(diff(idx) ~= 1); numel(idx)];
for b = 1:numel(brk) - 1
    r = idx(brk(b) + 1:brk(b + 1));
    if numel(r) < 30
        continue
    end
    axis_deg = L.angle_ref(r) - L.wall_ofs(r);
    d = L.dist(r);
    ds = [0; diff(d)];
    ylat = cumsum(ds .* sind(L.angle(r) - axis_deg));  % 左が +
    sides = {'L', L.ad_l(r), th_l, 1.0; 'R', L.ad_r(r), th_r, -1.0};
    for s = 1:2
        a = sides{s, 2};
        ok = a > sides{s, 3};
        if nnz(ok) < 20 || (max(ylat(ok)) - min(ylat(ok))) < min_range_mm
            continue
        end
        pf = polyfit(ylat(ok), a(ok), 1);
        cc = corrcoef(ylat(ok), a(ok));
        sg = sides{s, 4};
        if sg * cc(1, 2) >= min_r && sg * pf(1) > 0
            found(end+1, :) = {L.time_s(r(1)), string(sides{s, 1}), pf(1), cc(1, 2)}; %#ok<AGROW>
        end
    end
end
if height(found) == 0
    return
end
ks = sort(abs(found.slope));
k = ks(floor(numel(ks) / 2) + 1);  % yuho_common と同じ(偶数なら上の方)
n = height(found);
end
