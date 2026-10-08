function tp = turn_params_from_h(p, kind)
%TURN_PARAMS_FROM_H params.h の値から、機体と同じ旋回の値(v, ω, α, 調整分)を作る。
%   l180 の omega は NaN(求める)。
switch kind
    case 's90'
        tp.v = p.SLALOM_V_MM_S; tp.omega = p.SLALOM_OMEGA_DPS; tp.alpha = p.SLALOM_ALPHA_DPS2;
    case 'l90'
        tp.v = p.FAST_LARGE90_V_MM_S; tp.omega = p.FAST_LARGE90_OMEGA_DPS; tp.alpha = p.FAST_LARGE90_ALPHA_DPS2;
    otherwise
        tp.v = p.FAST_LARGE180_V_MM_S; tp.omega = NaN; tp.alpha = p.FAST_LARGE180_ALPHA_DPS2;
end
names = yc.turn_kinds().(kind).adj;
tp.pre_adj = getd(p, names{1});
tp.post_adj = getd(p, names{2});
end

function v = getd(p, name)
if isfield(p, name)
    v = p.(name);
else
    v = 0.0;
end
end
