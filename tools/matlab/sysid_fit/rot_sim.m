function W = rot_sim(P, Vd, v, D, w0, dt)
% rot_sim  旋回のモデルで角速度をシミュレーションする(列ごとに 1 本．行が時間)
%
%   b_w dω/dt = V_d(t − 1ms) − D − F
%   dF/dt     = (v/σ)(h − F)，  h = A± asinh(κ/κ0(v))，κ = ω/v
%
%   P : bw，sigma，Ap，Am，k0v(速さの表)，k0(κ0 の表．その間は直線でつなぐ)
%   Vd，v : 実測の V_d [V] と速さ [m/s](N×M．足りない所は NaN)
%   D : 本ごとの D [V](1×M)．w0 : 始めの角速度 [rad/s](1×M)
%   W : シミュレーションした ω(N×M)

[N, M] = size(Vd);
V  = max(v, 0.05);
V(isnan(V)) = 0.05;
K0 = interp1(P.k0v, P.k0, min(max(V, P.k0v(1)), P.k0v(end)));   % κ0(v) は実測の v で先に計算する
a  = min(V*dt/P.sigma, 1);
Vd(isnan(Vd)) = 0;

W = nan(N, M);
w = w0;
F = hfun(P, w ./ V(1, :), K0(1, :));      % 始めは定常にいるとする
VdPrev = Vd(1, :);
for k = 1:N
    W(k, :) = w;
    h = hfun(P, w ./ V(k, :), K0(k, :));
    F = F + a(k, :) .* (h - F);
    w = w + dt/P.bw * (VdPrev - D - F);
    VdPrev = Vd(k, :);
end
W(isnan(v)) = NaN;
end

function h = hfun(P, kappa, k0)
A = P.Am + (P.Ap - P.Am) * (kappa >= 0);
h = A .* asinh(kappa ./ k0);
end
