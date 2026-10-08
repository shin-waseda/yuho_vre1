function sim = simulate_turn(kind, v, omega, alpha, pre, post, K, C, extra_mm)
%SIMULATE_TURN 左に曲がるとして、entry から 前のオフセット → 旋回 → 後ろのオフセット → extra_mm 直進
%   を 1ms 刻みで進める。機体の向き θ は機体と同じ台形で回し、位置は θ − β の向きに v で進める。
%   β はスリップアングル: dβ/dt = (K v[m/s] ω[rad/s] − β) / C (C = 0 なら β = K v ω)。+ で外へずれる。
%   sim: x, y, theta[deg], beta[deg], phase(0 前, 1 旋回, 2 後ろ, 3 延長), i_post_end(後ろのオフセットの終わりの番号)
if nargin < 7, K = 0.0; end
if nargin < 8, C = 0.0; end
if nargin < 9, extra_mm = 180.0; end
kd = yc.turn_kinds().(kind);
dt = 0.001;
N = 20000;
X = nan(N, 1); Y = X; TH = X; BE = X; PH = X;
x = kd.entry(1); y = kd.entry(2); th = 90.0; beta = 0.0;
n = 1;
X(1) = x; Y(1) = y; TH(1) = th; BE(1) = 0; PH(1) = 0;

straight(pre, 0);
prof = yc.profile_start(kd.angle, 0.0, omega, 0.0, alpha);
while ~prof.done
    prof = yc.profile_step(prof, dt);
    tick(prof.v, 1, dt);
end
straight(post, 2);
sim.i_post_end = n;
straight(extra_mm, 3);

sim.x = X(1:n); sim.y = Y(1:n); sim.theta = TH(1:n); sim.beta = rad2deg(BE(1:n)); sim.phase = PH(1:n);

    function tick(om, phase, h)
        target = K * (v * 1e-3) * deg2rad(om);
        if C <= 0.0
            beta = target;
        else
            beta = beta + (1.0 - exp(-h / C)) * (target - beta);
        end
        th = th + om * h;
        a = deg2rad(th) - beta;
        x = x + v * h * cos(a);
        y = y + v * h * sin(a);
        n = n + 1;
        if n > numel(X)
            X(end+N) = nan; Y(end+N) = nan; TH(end+N) = nan; BE(end+N) = nan; PH(end+N) = nan;
        end
        X(n) = x; Y(n) = y; TH(n) = th; BE(n) = beta; PH(n) = phase;
    end

    function straight(dist, phase)
        % 端の誤差が出ないよう、最後の1歩は残りの距離だけ進む
        dist = max(0.0, dist);
        n_full = floor(dist / (v * dt));
        for k = 1:n_full
            tick(0.0, phase, dt);
        end
        rest = dist - n_full * v * dt;
        if rest > 1e-9
            tick(0.0, phase, rest / v);
        end
    end
end
