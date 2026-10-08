function study4_pi_design(K, T, lambda)
%STUDY4_PI_DESIGN 演習4: モデルから速度の PI のゲインを計算し、今のゲインと比べる(Control System Toolbox を使う)。
%   プラント  G(s) = K / (T s + 1)     (演習1〜2で求めた K, T．摩擦は FF で打ち消してあるとする)
%   PI       C(s) = Kp + Ki / s
%   IMC(内部モデル制御)の考え方: 「閉ループを時定数 λ の1次遅れにしたい」と決めると
%       Kp = T / (K λ),   Ki = 1 / (K λ)      (PI の零点 Ki/Kp = 1/T がプラントの極を打ち消す)
%   になる。λ を小さくするほど速いが、モデルにない遅れ(制御周期・PWM・ねじれ)で不安定に近づく。
%
%   study4_pi_design                     % 左の車輪の値(params.h の FF から)，λ = 20ms
%   study4_pi_design(2000, 0.55, 0.01)
p = yc.read_params();
if nargin < 1 || isempty(K), K = 1 / p.VELOCITY_FF_GAIN_L; end
if nargin < 2 || isempty(T), T = p.VELOCITY_FF_ACC_L / p.VELOCITY_FF_GAIN_L; end
if nargin < 3 || isempty(lambda), lambda = 0.02; end

s = tf('s');
G = K / (T * s + 1);
G.InputDelay = 0.001;            % 制御周期 1ms ぶんの遅れ(計算してから出すまで)を足しておく
Kp_now = p.VELOCITY_KP; Ki_now = p.VELOCITY_KI;
Kp_imc = T / (K * lambda); Ki_imc = 1 / (K * lambda);

fprintf('プラント K = %.0f mm/s/V, T = %.3f s\n', K, T);
fprintf('今のゲイン     Kp = %.4f, Ki = %.4f  → 零点 Ki/Kp = %.2f rad/s (1/T = %.2f)\n', Kp_now, Ki_now, Ki_now / Kp_now, 1 / T);
fprintf('IMC(λ=%.0fms) Kp = %.4f, Ki = %.4f\n', lambda * 1000, Kp_imc, Ki_imc);
fprintf('今のゲインを IMC と見ると λ = T/(K·Kp) = %.1f ms (Kp から), 1/(K·Ki) = %.1f ms (Ki から)\n', ...
    T / (K * Kp_now) * 1000, 1 / (K * Ki_now) * 1000);

figure('Name', '演習4: モデルから PI を決める', 'Color', 'w');
tl = tiledlayout(1, 2, 'TileSpacing', 'compact');
nexttile(tl);
C_now = pid(Kp_now, Ki_now); C_imc = pid(Kp_imc, Ki_imc);
step(feedback(C_now * G, 1), feedback(C_imc * G, 1), 0.15);
legend('今のゲイン', sprintf('IMC λ=%.0fms', lambda * 1000), 'Location', 'southeast');
title('目標の速さを 1 だけ上げたときの応答(FF なし．PI だけ)');
grid on
nexttile(tl);
margin(C_now * G);
grid on
[gm, pm, ~, wc] = margin(C_now * G);
fprintf('今のゲインの余裕: 位相余裕 %.0f°, ゲイン余裕 %.1f 倍, 交差 %.0f rad/s\n', pm, gm, wc);
fprintf('(位相余裕の目安は 45〜60°。小さいと行き過ぎて揺れる)\n');
end
