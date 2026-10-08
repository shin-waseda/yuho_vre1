function m = study2_arx(files)
%STUDY2_ARX 演習2: 離散時間の1次モデル(ARX)を最小二乗法で求め、時定数 T とゲイン K に直す。
%   モデル:  v[k] = α·v[k−1] + β·u[k−1] + γ·sign(v[k−1])
%   連続時間の  T·dv/dt + v = K·(u − FRIC·sign(v))  をサンプル間隔 h で離散化すると(ゼロ次ホールド)
%       α = exp(−h/T),  β = K·(1 − α),  γ = −β·FRIC
%   になるので、求めた α, β, γ から  T = −h/ln(α),  K = β/(1 − α),  FRIC = −γ/β  に戻せる。
%
%   演習1との違い: 演習1は加速度(速さの微分)を使ったが、こちらは微分を使わない(雑音に強い)。
%   代わりに「前の時刻の速さ」を使う。同じモデルを違う方法で求めて、答えがそろうかを見るのが大事。
if nargin < 1 || isempty(files)
    files = study_files();
end
names = {'l', 'r'};
m = struct();
for w = 1:2
    s = names{w};
    D = study_data(files, s);
    % 隣り合う2行がどちらも使える所だけ(ログの境目はまたがない)
    k = find(D.ok(2:end) & D.ok(1:end-1) & D.file(2:end) == D.file(1:end-1)) + 1;
    h = median(D.dt);  % サンプル間隔(ログはどれも同じ間隔とする)
    X = [D.v(k - 1), D.u(k - 1), sign(D.v(k - 1))];
    y = D.v(k);
    th = X \ y;
    al = th(1); be = th(2); ga = th(3);
    T = -h / log(al);
    K = be / (1 - al);
    F = -ga / be;
    yhat = X * th;
    fprintf('\n[%s] 1歩先の予測(h = %.0f ms): α = %.4f, β = %.3f, γ = %.3f\n', upper(s), h * 1000, al, be, ga);
    fprintf('  → T = %.3f s, K = %.0f mm/s/V, FRIC = %.3f V   (1歩先の予測の誤差 RMS %.1f mm/s)\n', ...
        T, K, F, sqrt(mean((y - yhat).^2)));
    fprintf('  演習1の形に直すと GAIN = 1/K = %.6f, ACC = T/K = %.7f\n', 1 / K, T / K);
    m.(s) = struct('T', T, 'K', K, 'FRIC', F, 'h', h);
end
fprintf(['\n注意: ログの間隔 h は 5ms(ロガーの間引き)で、制御の 1ms より粗い。また機体は PI で速さを制御しているので、\n' ...
    '電圧 u は速さ v に応じて変わる(閉ループのデータ)。最小二乗法はこのとき少しずれた答えを出すことがある(偏り)。\n']);
end
