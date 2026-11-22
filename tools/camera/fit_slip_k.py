#!/usr/bin/env python3
"""SLIP の CAMERA で撮った止まった位置から、スリップアングルの K(と C)を当てはめる(2026-11-20)。

- 入力: analyze_slip_camera.m が書いた slip_camera.csv(いくつでも。TURN・LARGE90・LARGE180 をまとめて当てはめられる)。
- 機体はスリップのモデルを切って走る(App_SetSlipModel(false))ので、止まった位置のずれ = 滑りの分 + 速さによらない分。
- 止まった位置を、曲がった後の進む向き(前後 along)と外(曲がる中心の反対、横 lat)に直す。
- モデル: lat = b_lat[走り] + Δlat(v; K, C)、along = b_along[走り] + Δalong(v; K, C)。
  Δ は yuho_common.simulate_turn(機体と同じ台形)で、K, C ありとなし(K = 0)の止まる所の差。
  b は走り(T1・T1L・L90R…)ごとの定数(置き方・壁・調整のずれ)で、K, C ごとに最小二乗で決める。
  つまり K は「速さを変えたときにずれがどう変わるか」だけで決まる。
- 既定は横(lat)だけで当てはめる(前後は止まり方・車輪の滑りなど、ほかの速さによる分も入るので確かめに使う)。

使い方:
    python tools/camera/fit_slip_k.py photos/slip_turn/slip_camera.csv photos/slip_l90/slip_camera.csv
    python tools/camera/fit_slip_k.py ... --fit-c          # C も探す(既定は params.h の SLALOM_SLIP_C_S に固定)
    python tools/camera/fit_slip_k.py ... --use both       # 横と前後の両方で当てはめる
    python tools/camera/fit_slip_k.py ... --plot out.png   # 速さごとのずれとモデルの図
"""

import argparse
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import yuho_common as yc  # noqa: E402

# 走りの名前 → (旋回の種類, 進む向き along の単位ベクトル, 外 lat の単位ベクトル)(迷路の座標: 東 +x、北 +y)
RUNS = {
    "T1": ("s90", (1, 0), (0, 1)),      # S から右 → 東へ。曲がる中心は南
    "T1L": ("s90", (-1, 0), (0, 1)),    # (2,0) から左 → 西へ。中心は南
    "L90R": ("l90", (1, 0), (0, 1)),
    "L90L": ("l90", (-1, 0), (0, 1)),
    "L180R": ("l180", (0, -1), (1, 0)),  # (0,2) から右 180° → 南へ。中心は西
    "L180L": ("l180", (0, -1), (-1, 0)),  # (3,2) から左 180° → 南へ。中心は東
}

FIT_K = np.arange(0.0, 0.0301, 0.0002)
FIT_C = np.array([0.0, 0.005, 0.01, 0.02, 0.03, 0.05, 0.08, 0.12, 0.2])


def load_rows(paths, min_leds, max_dth):
    rows = []
    for p in paths:
        with open(p, newline="", encoding="utf-8-sig") as f:
            for r in csv.DictReader(f):
                name = r["name"]
                if name not in RUNS:
                    print(f"{p}: 知らない走り {name} は飛ばす")
                    continue
                if int(float(r["n_leds_used"])) < min_leds:
                    print(f"{p}: #{r['run']} {r['file']} は LED が {r['n_leds_used']} 個なので飛ばす")
                    continue
                _, ua, ul = RUNS[name]
                x, y = float(r["x_mm"]), float(r["y_mm"])
                dth = float(r["dth_deg"])
                if not abs(dth) <= max_dth:
                    print(f"{p}: #{r['run']} の止まった所は向きが {dth:+.1f}° ずれているので飛ばす(LED の取り違え)")
                    continue
                x0 = float(r.get("x0_mm") or "nan")
                if "x0_mm" in r and not math.isfinite(x0):
                    # 置いた所を撮る形なのに置いた所が出なかった(ほかの本と同じ物差しで比べられない)
                    print(f"{p}: #{r['run']} は置いた所の位置が出なかったので飛ばす")
                    continue
                if math.isfinite(x0):
                    # 置いた所の写真があるとき: 止まった所を、置いた所から見た位置にし、置いた向きを北(90°)に回す
                    # (手で置いた位置と向きのずれを消す。どの走りも北向きに置く)
                    if int(float(r["n0_leds_used"])) < min_leds:
                        print(f"{p}: #{r['run']} の置いた所は LED が {r['n0_leds_used']} 個なので飛ばす")
                        continue
                    y0, th0 = float(r["y0_mm"]), float(r["th0_deg"])
                    if not abs(th0 - 90.0) <= max_dth:
                        print(f"{p}: #{r['run']} の置いた所は向きが {th0:.1f}° なので飛ばす(LED の取り違え)")
                        continue
                    a = math.radians(90.0 - th0)
                    dx, dy = x - x0, y - y0
                    x, y = dx * math.cos(a) - dy * math.sin(a), dx * math.sin(a) + dy * math.cos(a)
                    dth += 90.0 - th0
                rows.append(dict(name=name, v=float(r["v"]), run=int(float(r["run"])), file=r["file"],
                                 along=x * ua[0] + y * ua[1], lat=x * ul[0] + y * ul[1], dth=dth))
    return rows


def turn_at_speed(params, kind, v):
    """機体と同じ形(基の速さの形を、角速度は速さに比例、角加速度は 2 乗に比例させる)の (omega, alpha, pre, post)。"""
    tp = yc.turn_params_from_h(params, kind)
    k = v / tp["v"]
    alpha = tp["alpha"] * k * k
    omega = None if tp["omega"] is None else tp["omega"] * k
    omega, pre, post, _ = yc.turn_offsets(kind, v, omega, alpha)
    return omega, alpha, pre + tp["pre_adj"], post + tp["post_adj"]


def slip_delta(params, kind, v, K, C, cache):
    """K, C ありとなしの止まる所の差(along, lat。外が +)。曲がった後の直進で β が収まった後の値。"""
    key = (kind, v, round(K, 6), round(C, 6))
    if key not in cache:
        omega, alpha, pre, post = turn_at_speed(params, kind, v)
        s1 = yc.simulate_turn(kind, v, omega, alpha, pre, post, K=K, C=C)
        s0 = yc.simulate_turn(kind, v, omega, alpha, pre, post, K=0.0, C=0.0)
        kd = yc.TURN_KINDS[kind]
        ux, uy = kd["exit_dir"]
        nx, ny = uy, -ux  # 左に曲がったときの外
        dx, dy = s1["x"][-1] - s0["x"][-1], s1["y"][-1] - s0["y"][-1]
        cache[key] = (dx * ux + dy * uy, dx * nx + dy * ny)
    return cache[key]


def fit(rows, params, Ks, Cs, use, cache):
    """全部の (K, C) で走りごとの定数を最小二乗で決め、残りの二乗和が一番小さい組を返す。"""
    comps = ["lat"] if use == "lat" else ["along"] if use == "along" else ["lat", "along"]
    best = None
    table = np.full((len(Cs), len(Ks)), np.nan)
    for ic, C in enumerate(Cs):
        for ik, K in enumerate(Ks):
            groups = defaultdict(list)
            for r in rows:
                da, dl = slip_delta(params, RUNS[r["name"]][0], r["v"], K, C, cache)
                groups[r["name"]].append((r["along"] - da, r["lat"] - dl))
            sse = 0.0
            n = 0
            for g in groups.values():
                a = np.array(g)
                for c in comps:
                    col = a[:, 0] if c == "along" else a[:, 1]
                    sse += float(np.sum((col - col.mean()) ** 2))
                    n += len(col)
            table[ic, ik] = sse
            if best is None or sse < best[0]:
                best = (sse, K, C, n)
    return best, table


def summarize(rows, params, K, C, cache):
    """走り・速さごとの平均とばらつき、モデルの値(定数込み)を出す。"""
    groups = defaultdict(list)
    for r in rows:
        groups[r["name"]].append(r)
    out = []
    for name, rs in groups.items():
        kind = RUNS[name][0]
        d = [slip_delta(params, kind, r["v"], K, C, cache) for r in rs]
        b_along = float(np.mean([r["along"] - da for r, (da, _) in zip(rs, d)]))
        b_lat = float(np.mean([r["lat"] - dl for r, (_, dl) in zip(rs, d)]))
        for v in sorted({r["v"] for r in rs}):
            sel = [r for r in rs if r["v"] == v]
            da, dl = slip_delta(params, kind, v, K, C, cache)
            a = np.array([r["along"] for r in sel])
            l = np.array([r["lat"] for r in sel])
            th = np.array([r["dth"] for r in sel])
            omega, alpha, _, _ = turn_at_speed(params, kind, v)
            ay_g = v * 1e-3 * math.radians(omega) / 9.80665
            out.append(dict(name=name, v=v, n=len(sel), ay_g=ay_g,
                            lat=l.mean(), lat_sd=l.std(ddof=1) if len(l) > 1 else 0.0, lat_model=b_lat + dl,
                            along=a.mean(), along_sd=a.std(ddof=1) if len(a) > 1 else 0.0, along_model=b_along + da,
                            dth=th.mean(), b_lat=b_lat, b_along=b_along))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+", type=Path, help="analyze_slip_camera.m の slip_camera.csv")
    ap.add_argument("--use", choices=["lat", "along", "both"], default="lat", help="当てはめに使うずれ(既定 lat)")
    ap.add_argument("--fit-c", action="store_true", help="C も探す(既定は SLALOM_SLIP_C_S に固定)")
    ap.add_argument("--min-leds", type=int, default=3, help="当てはめた LED がこれより少ない写真は使わない(既定 3)")
    ap.add_argument("--max-dth", type=float, default=10.0,
                    help="置いた所・止まった所の向きが決めた向きからこれ以上ずれた写真は使わない[deg](既定 10)")
    ap.add_argument("--plot", type=Path, help="図を書く PNG")
    a = ap.parse_args()

    params = yc.read_params()
    rows = load_rows(a.csv, a.min_leds, a.max_dth)
    if not rows:
        sys.exit("使える行がない")
    print(f"{len(rows)} 本: " + ", ".join(f"{n} {sum(r['name'] == n for r in rows)}" for n in RUNS
                                          if any(r["name"] == n for r in rows)))
    c_now = params.get("SLALOM_SLIP_C_S", 0.02)
    Cs = FIT_C if a.fit_c else np.array([c_now])
    cache = {}
    (sse, K, C, n), table = fit(rows, params, FIT_K, Cs, a.use, cache)
    n_par = 1 + (1 if a.fit_c else 0) + len({r["name"] for r in rows}) * (2 if a.use == "both" else 1)
    rms = math.sqrt(sse / max(1, n - n_par))
    # K のだいたいの幅: 残りの二乗和が 1 自由度ぶん(rms² だけ)増える所まで
    ic = int(np.where(Cs == C)[0][0])
    ok = FIT_K[table[ic] <= sse + rms * rms]
    print(f"\nK = {K:.4f}(幅 {ok.min():.4f}〜{ok.max():.4f})  C = {C:.3f} s{'(固定)' if not a.fit_c else ''}  "
          f"使ったずれ {a.use}  残り RMS {rms:.2f} mm  (params.h は K {params.get('SLALOM_SLIP_K', float('nan'))})")
    if a.fit_c:
        print("C ごとの一番よい K と残りの二乗和:")
        for i, c in enumerate(Cs):
            j = int(np.argmin(table[i]))
            print(f"  C {c:.3f}: K {FIT_K[j]:.4f}  SSE {table[i, j]:.1f}")
    k0 = np.argmin(np.abs(FIT_K - 0.0))
    print(f"K = 0(滑りなし)の残りの二乗和 {table[ic, k0]:.1f} → {sse:.1f}")

    summ = summarize(rows, params, K, C, cache)
    print("\n走り  速さ  本  横加速度  横(外 +) 平均 ±sd / モデル   前後 平均 ±sd / モデル   向きのずれ   [mm, deg]")
    for s in summ:
        print(f"{s['name']:6s}{s['v']:5.0f} {s['n']:3d}  {s['ay_g']:5.2f}G   "
              f"{s['lat']:7.2f} ±{s['lat_sd']:4.2f} / {s['lat_model']:7.2f}   "
              f"{s['along']:7.2f} ±{s['along_sd']:4.2f} / {s['along_model']:7.2f}   {s['dth']:+6.2f}")
    print("\n走りごとの定数(速さによらない分): " + ", ".join(
        f"{n} 横 {s['b_lat']:+.1f} 前後 {s['b_along']:+.1f}" for n, s in {s['name']: s for s in summ}.items()))

    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        yc.setup_japanese_font()
        fig, axs = plt.subplots(1, 2, figsize=(11, 4.5))
        for ax, comp in zip(axs, ["lat", "along"]):
            for name in RUNS:
                ss = [s for s in summ if s["name"] == name]
                if not ss:
                    continue
                kind = RUNS[name][0]
                b = ss[0]["b_" + comp]
                pts = [r for r in rows if r["name"] == name]
                line = ax.plot([r["v"] for r in pts], [r[comp] - b for r in pts], "o", ms=4, alpha=0.6, label=name)[0]
                vs = np.linspace(min(s["v"] for s in ss), max(s["v"] for s in ss), 20)
                ax.plot(vs, [slip_delta(params, kind, float(v), K, C, cache)[0 if comp == "along" else 1] for v in vs],
                        "-", color=line.get_color())
            ax.set_xlabel("速さ [mm/s]")
            ax.set_ylabel(("横(外 +)" if comp == "lat" else "前後(先 +)") + " − 走りの定数 [mm]")
            ax.grid(alpha=0.3)
        axs[0].legend()
        fig.suptitle(f"K = {K:.4f}, C = {C:.3f} s(線はモデル、点は写真)")
        fig.tight_layout()
        fig.savefig(a.plot, dpi=120)
        print(f"wrote {a.plot}")


if __name__ == "__main__":
    main()
