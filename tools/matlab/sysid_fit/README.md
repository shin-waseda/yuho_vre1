# tools/matlab/sysid_fit — 同定の実験のログへのモデルの当てはめ(MATLAB)

MathWorks 賞の応募資料(`docs/mathworks_award/`)のために，同定の実験(E1S・E1T3・E1T・E1C)のログに，
並進と旋回のモデル(`docs/yuho_model.md` の形)を MATLAB で当てはめ直したもの．以前の値(機体で使っている値)は，Python でログを解析しながら感覚で合わせたもの．

- ログは `yuho-sysid` の worktree(`feature/system-identification`)の `logs/sysid/` を読む(`sf_config.m`)．
- 使う関数は MATLAB 本体だけ(`fminsearch`)．Toolbox は要らない．
- 図は `docs/mathworks_award/fig/`，結果の `.mat` は `out/` に書く．

```matlab
cd tools/matlab/sysid_fit
fit_translation            % 並進: 床・浮かせたの電圧のステップ(E1S)→ a，V0，b(約 1 分)
fit_rotation('const')      % 旋回: 電圧の差のステップ(E1T3，E1T 0.3m/s)→ b_w，σ，A±，κ0(v)，D(約 2 分)
fit_rotation('run')        %   D を本ごとの V_d0 にした場合(比べるため)
validate_turns('const')    % 当てはめに使っていない FF だけのターン(E1C)で確かめる
sigma_check                % σ だけ変えたときの誤差(σ が決まるか)
shot_viewer('M:/User/STM32CubeIDE/workspace_1.19.0/yuho/logs/search/search_0098.bin', 'viewer_search.png')
```

## やり方(出力誤差法)

実際にかけた電圧(pwm/4199 × vbat)をモデルに入れて速さ・角速度をシミュレーションし，全部の本の誤差をまとめて `fminsearch` で最小にする．
本ごとに長さが違うので，NaN で埋めた行列(列が 1 本，行が時間)にして全部の本を 1 度に時間発展させる．

## 結果(2026-11-13)

| | MATLAB | 前の値 |
|---|---|---|
| 床 a / V0 / b | 0.595 / 0.203 / 0.322 | 0.561 / 0.210 / 0.32 |
| 浮かせた a / V0 / τ | 0.559 / 0.080 / 0.082 s | 0.558 / 0.077 / 0.082 s |
| b_w / σ / A+ / A− / D | 8.76e-3 / 1.60 mm / 0.334 / 0.301 / −0.063 | 9.20e-3 / 3.02 mm / 0.32 / 0.216 / −0.128 |
| κ0(0.3/0.5/1.0/1.5 m/s) | 1.55 / 1.83 / 1.48 / 1.42 | 1.03 / 1.14 / 0.89 / 0.92 |
| E1C の一致度(中央値) / 一定の区間の比 | 63.4 % / 0.91 ± 0.12 | 62.4 % / 0.94 ± 0.09(D = −0.128) |

- 浮かせた状態は 0.4 V 以下で直線から外れるので，0.45 V 以上だけを使った．
- D を本ごとの V_d0 にすると E1C の一致度が 35 % に落ちる．
- σ は 0.5〜3 mm で誤差がほとんど変わらない(決まらない)．
- 機体の `params.h` はこの値に変えていない．
