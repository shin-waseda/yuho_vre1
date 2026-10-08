function [omega, pre, post, sh] = turn_offsets(kind, v, omega, alpha)
%TURN_OFFSETS 機体と同じ前後のオフセット(調整分を足す前)。l180 は omega を求め直す。
c = yc.consts();
angle = yc.turn_kinds().(kind).angle;
if strcmp(kind, 'l180')
    % Slalom_SolveOmegaForSide と同じ(横が 1区画になる ω を二分法 30 回)
    lo = 30.0; hi = 3000.0;
    for k = 1:30
        om = 0.5 * (lo + hi);
        s = yc.slalom_shape(v, om, alpha, angle);
        if s(2) > c.SECTION_MM
            lo = om;
        else
            hi = om;
        end
    end
    omega = 0.5 * (lo + hi);
    sh = yc.slalom_shape(v, omega, alpha, angle);
    pre = c.SECTION_MM - sh(3);
    post = pre + sh(1);
else
    sh = yc.slalom_shape(v, omega, alpha, angle);
    if strcmp(kind, 's90')
        span = 0.5 * c.SECTION_MM;
    else
        span = c.SECTION_MM;
    end
    pre = span - sh(1);
    post = span - sh(2);
end
end
