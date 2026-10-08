function p = profile_step(p, dt)
%PROFILE_STEP 機体の VelocityProfile_Step と同じ計算で1歩進める。
MARGIN = 1.5;  % PROFILE_DECEL_MARGIN
if p.done
    p.v = p.v_end;
    p.a = 0.0;
    return
end
remaining = p.distance - p.pos;
if ~p.decelerating
    decel_dist = 0.0;
    if p.v > p.v_end
        decel_dist = (p.v * p.v - p.v_end * p.v_end) / (2.0 * p.accel);
    end
    if remaining - p.v * dt <= decel_dist
        p.decelerating = true;
    end
end
if p.decelerating
    if remaining > 1e-6 && p.v > p.v_end
        a_cmd = -(p.v * p.v - p.v_end * p.v_end) / (2.0 * remaining);
        a_cmd = max(a_cmd, -MARGIN * p.accel);
    else
        a_cmd = 0.0;
    end
elseif p.v < p.v_max
    a_cmd = p.accel;
else
    a_cmd = 0.0;
end
v_next = p.v + a_cmd * dt;
if a_cmd > 0.0 && v_next > p.v_max
    v_next = p.v_max;
end
if a_cmd < 0.0 && v_next < p.v_end
    v_next = p.v_end;
end
if v_next < 0.0
    v_next = 0.0;
end
p.pos = p.pos + 0.5 * (p.v + v_next) * dt;
p.a = (v_next - p.v) / dt;
p.v = v_next;
reached = p.pos >= p.distance - 1e-3;
slowed = p.decelerating && p.v <= p.v_end && (p.v_end > 0.0 || p.v <= 0.0);
if reached || slowed
    p.done = true;
    p.v = p.v_end;
    p.a = 0.0;
end
end
