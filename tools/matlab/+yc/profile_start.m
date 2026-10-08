function p = profile_start(distance, v_start, v_max, v_end, accel)
%PROFILE_START 機体の VelocityProfile_Start と同じ(logic/control/velocity_profile.c)。
%   p = yc.profile_start(...); while ~p.done, p = yc.profile_step(p, dt); end
p.distance = distance;
p.v_max = v_max;
p.v_end = min(v_end, v_max);
p.accel = accel;
p.pos = 0.0;
p.v = v_start;
p.a = 0.0;
p.decelerating = false;
p.done = distance <= 0.0 || accel <= 0.0;
end
