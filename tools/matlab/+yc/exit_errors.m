function e = exit_errors(kind, sim)
%EXIT_ERRORS 後ろのオフセットの後のずれ。横は外向き(曲がる中心の反対)が +、前後は出る向きが +。
%   lat_post: 後ろのオフセットの後、lat_final: 延長の直進の後(スリップが収まった後)
kd = yc.turn_kinds().(kind);
ex = kd.exit(1); ey = kd.exit(2);
ux = kd.exit_dir(1); uy = kd.exit_dir(2);
nx = uy; ny = -ux;  % 左に曲がったとき、出る向きの右 = 外
i = sim.i_post_end;
dx = sim.x(i) - ex; dy = sim.y(i) - ey;
dxf = sim.x(end) - ex; dyf = sim.y(end) - ey;
e.along = dx * ux + dy * uy;
e.lat_post = dx * nx + dy * ny;
e.lat_final = dxf * nx + dyf * ny;
e.travel_err = sim.theta(i) - sim.beta(i) - (90.0 + kd.angle);
e.ymax = max(sim.y);
end
