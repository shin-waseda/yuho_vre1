function sh = slalom_shape(v, omega, alpha, angle)
%SLALOM_SHAPE Slalom_ComputeShape と同じ。sh = [forward, side, forward_max, length, time]
dt = 0.001;  % CONTROL_DT_S
prof = yc.profile_start(angle, 0.0, omega, 0.0, alpha);
theta = 0.0;
fwd = 0.0; side = 0.0; fmax = 0.0;
ticks = 0;
while ~prof.done && ticks < 100000
    prof = yc.profile_step(prof, dt);
    theta = theta + prof.v * dt;
    fwd = fwd + v * cosd(theta) * dt;
    side = side + v * sind(theta) * dt;
    fmax = max(fmax, fwd);
    ticks = ticks + 1;
end
t = ticks * dt;
sh = [fwd, side, fmax, v * t, t];
end
