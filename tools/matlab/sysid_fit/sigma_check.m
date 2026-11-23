function T = sigma_check()
% sigma_check  σ だけを変えたとき(ほかは当てはめた値のまま)の誤差を見る(σ が決まるかどうか)
%
%   T = sigma_check      % out/rotation_const.mat の値の周りで

cfg = sf_config();
s = load(fullfile(cfg.outDir, 'rotation_const.mat'));
P = s.R.P;  S = s.R.S;
sig = [0.5 1 1.6 2 3 5 8] * 1e-3;
J = zeros(size(sig));
for i = 1:numel(sig)
    Q = P;  Q.sigma = sig(i);
    W = rot_sim(Q, S.Vd, S.v, Q.D * ones(1, numel(S.D)), S.w0, cfg.dt);
    E = W(1:end-1, :) - S.w(2:end, :);
    e = sqrt(mean(E.^2, 1, 'omitnan')) ./ S.scale;
    J(i) = mean(e.^2);
end
T = table(1e3*sig', J', 100*(J'/min(J) - 1), 'VariableNames', {'sigma_mm', 'J', 'increase_pct'});
disp(T);
end
