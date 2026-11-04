# yuho の機体モデル（11-03 時点）

このファイルには，FF とシミュレーションに使うための式と値だけをまとめた．どう求めたか，どう確かめたかは `yuho_model_v2.md` と `sysid_v2.md` に書いてある．

## 1. 使える範囲

| 項目 | 範囲 |
|---|---|
| 機体 | yuho（4 輪．片側 1 モーターで前後輪を駆動．吸引なし） |
| 向き | 前進のみ（$v > 0$） |
| 並進の速さ | 0.3〜2.0 m/s |
| 加減速度 | 5 m/s² まで |
| 曲がり具合 | $\lvert\kappa\rvert$ ≤ 25 /m |
| 角加速度 | 約 8000 dps² まで（それより強いと，角加速の終わりに 8〜11 Hz の外れが 1 周期出る） |
| タイヤ | 毎回掃除する |

- 単位は SI（V，m/s，rad/s）．
- 制御の周期は 1 ms．

## 2. 記号

| 記号 | 意味 |
|---|---|
| $V_L$，$V_R$ | 左右のモーターの電圧 [V]．duty = $V / V_{bat}$ |
| $V_s = (V_L + V_R)/2$ | 並進の電圧 |
| $V_d = (V_R - V_L)/2$ | 旋回の電圧（左回りが正） |
| $v$ | 並進の速さ（エンコーダーの左右平均）[m/s] |
| $\omega$ | 角速度（ジャイロ z，左回りが正）[rad/s] |
| $\kappa = \omega / v$ | 曲がり具合 [1/m] |
| $F$ | タイヤが横にこする分を電圧で表した状態 [V] |

電圧の差と和から戻すときは，$V_L = V_s - V_d$，$V_R = V_s + V_d$．

## 3. 式

状態は $x = [v,\ \omega,\ F]$，入力は $u = [V_L,\ V_R]$．

$$b\,\dot v = V_s - a\,v - V_0 - c\,v\,\omega^2$$

$$b_\omega\,\dot\omega(t) = V_d(t - t_d) - D - F$$

$$\dot F = \frac{v}{\sigma}\,\big(h - F\big),\qquad h = A_\pm\,\mathrm{asinh}\!\left(\frac{\kappa}{\kappa_0(v)}\right)$$

- $A$ には，$\kappa \ge 0$（左回り）のとき $A_+$，$\kappa < 0$（右回り）のとき $A_-$ を使う．
- 出力は $y_1 = v$ と $y_2 = \omega(t - t_g)$（ジャイロ）．

**状態方程式**

$$\dot v = -1.753\,v + 1.563\,V_L + 1.563\,V_R - 0.656 - 0.0128\,v\,\omega^2$$

$$\dot\omega = 54.35\,(V_R - V_L)_{t - t_d} - 108.7\,(D + F)$$

## 4. 係数

| 係数 | 値 |
|---|---|
| $a$ | 0.561 V/(m/s) |
| $V_0$ | 0.210 V |
| $b$ | 0.32 V/(m/s²) |
| $c$ | 0.0041 V/(m/s·(rad/s)²) |
| $b_\omega$ | 9.20×10⁻³ V/(rad/s²) |
| $\sigma$ | 3.02 mm |
| $A_+$ / $A_-$ | 0.32 / 0.216 V |
| $\kappa_0(v)$ | 1.03 / 1.14 / 0.80 / 0.89 / 0.92 /m |
| $D$ | −0.128 V（本ごとに ±0.05 V ばらつく） |
| $t_d$ | 1 ms（入力の遅れ） |
| $t_g$ | 1.2 ms（ジャイロの遅れ） |

- $\kappa_0(v)$ の値は，$v$ = 0.3 / 0.5 / 0.75 / 1.0 / 1.5 m/s のときのもの．その間は直線でつなぐ．
- 時定数の目安：
  - 並進：$b/a$ = 0.57 s
  - $F$：$\sigma/v$ = 1.5〜10 ms

## 5. シミュレーション（1 ms ごと）

```
V_d_in = V_d[k-1]                      # t_d = 1 ms
kappa  = omega / max(v, 0.05)
A      = A_plus if kappa >= 0 else A_minus
h      = A * asinh(kappa / kappa0(v))
F     += min(v * dt / sigma, 1) * (h - F)
omega += dt / b_omega * (V_d_in - D - F)
v     += dt / b * (V_s - a*v - V0 - c*v*omega**2)
gyro   = omega を 1.2 ms 遅らせたもの
```

## 6. FF（モデルの逆）

目標の $v_{ref}$，$a_{ref}$，$\omega_{ref}$，$\dot\omega_{ref}$ から，1 ms ごとに次を計算する．

1. $\kappa_{ref} = \omega_{ref}/v_{ref}$，$h = A_\pm\,\mathrm{asinh}(\kappa_{ref}/\kappa_0(v_{ref}))$
2. $F_{ff} \leftarrow F_{ff} + \min(v_{ref}\Delta t/\sigma,\ 1)\,(h - F_{ff})$
   - ターンの初めは $F_{ff} = 0$ にする．
3. $V_{d,ff} = b_\omega\,\dot\omega_{ref} + D + F_{ff}$
   - $D$ は −0.128 V でもよい．直進のときに測った $V_d$ の平均 $V_{d0}$ を使ってもよい．
4. $V_{s,ff} = a\,v_{ref} + V_0 + b\,a_{ref} + c\,v_{ref}\,\omega_{ref}^2$

## 7. FB

| | 式 | 値 |
|---|---|---|
| 並進（PI） | $V_s = V_{s,ff} + K_p e + K_i \int e\,dt$，$e = v_{ref} - v$ | $K_p$ = 5.8 V/(m/s)，$K_i$ = 29 V/m，積分の上限 ±0.5 V |
| 旋回（PD，積分なし） | $V_d = V_{d,ff} + K_\omega(\omega_{ref} - \omega) + K_\theta(\theta_{ref} - \theta)$ | $K_\theta$ = 8.28 V/rad（0.1445 V/deg），$K_\omega$ = 0.386 V/(rad/s)（6.74×10⁻³ V/dps），FB の項は ±1.5 V で切る |

- 旋回の FB は，$\omega_n$ = 30 rad/s，$\zeta$ = 0.7 から決めた．
- 遅れを 3.2 ms とした計算では，位相余裕 56.7°，ゲイン余裕 21.1 dB．
- $\omega$ にフィルタはかけない．

## 8. モデルに入っていないもの

- 8〜11 Hz の外れ：角加速度が約 8000 dps² を超えたとき．
- 30 Hz の揺れ：一定の区間の $\omega$ に，std 16〜58 dps．
- 本ごとのばらつき：$D$ は ±0.05 V，$A_+$ は日によって 0.30〜0.35 V．
- 扱っていない動き：後退，超信地，0.3 m/s より遅い所，$\lvert\kappa\rvert$ > 25 /m．
