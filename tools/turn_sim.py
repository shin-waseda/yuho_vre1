#!/usr/bin/env python3
"""yuho のスラロームのシミュレータ(turn_sim_classic2.py の作り直し)。

- 機体と同じ計算: params.h を読み、logic/control/slalom.c と同じ台形・同じ前後のオフセット
  (小回り 90°、大回り 90°、大回り 180°)で曲がる。
- スリップアングル: 進む向き = 機体の向き − β。β は 1次遅れ
      dβ/dt = (K × v[m/s] × ω[rad/s] − β) / C        (C = 0 なら β = K v ω)
  β は外(曲がる中心の反対)へずれる向きを + とする。
- 結果: 機体と同じ前後のオフセット、出口のずれ(前後・横)、柱との距離、
  ずれを消す PRE/POST_ADJ(params.h に貼れる形)。
- 実機のログ(SLALOM テストの logs/slalom/*.csv / .bin)を重ねる: 車輪の距離とジャイロの向きで軌道を作る。
  車輪とジャイロだけの軌道には横滑りが出ないので、曲がった後の横の壁センサー(L, R)で見た横のずれとの差を
  「スリップで横にずれた量」とし、それに合う K と C を探す。
  横のずれ[mm] = (センサーの値 − WALL_REF) / 傾き[AD/mm]。傾きは探索のログから見積もれる(だいたいの値)。

使い方:
    pip install matplotlib numpy
    python tools/turn_sim.py
"""

import math
import re
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle

import yuho_common as yc

LOG_DIR = yc.ROOT / "logs"

# 合わせるときに探す範囲
FIT_K = np.linspace(0.0, 0.03, 151)
FIT_C = np.concatenate([[0.0], np.linspace(0.002, 0.2, 100)])


# ---------------------------------------------------------------------------
# 実機のログ(SLALOM テスト)
# ---------------------------------------------------------------------------

class TurnLog:
    """SLALOM テストの1回分。左に曲がった形(右なら左右を反転)にそろえて持つ。"""

    def __init__(self, path: Path, params: dict):
        self.path = Path(path)
        self.name = self.path.stem
        log = yc.load_log(self.path)
        for k in ("time_s", "dist", "angle", "omega_ref", "ad_l", "ad_r"):
            if k not in log:
                raise ValueError(f"{self.name}: '{k}' の列がない(SLALOM テストのログではない?)")
        t = np.array(log["time_s"])
        ang = np.array(log["angle"])
        om_ref = np.array(log["omega_ref"])
        turning = np.where(np.abs(om_ref) > 0.5)[0]
        if len(turning) == 0:
            raise ValueError(f"{self.name}: 曲がっていない")
        self.right = ang[-1] < 0.0
        s = -1.0 if self.right else 1.0
        self.kind = self._detect_kind(om_ref, ang, params)
        self.i_turn0, self.i_turn1 = int(turning[0]), int(turning[-1])
        self.peak_omega = float(np.max(np.abs(om_ref)))

        log_l = dict(log)
        log_l["angle"] = list(s * ang)
        xs, ys = yc.reconstruct_path(log_l, *yc.TURN_KINDS[self.kind]["start"], 90.0)
        self.t = t
        self.x = np.array(xs)
        self.y = np.array(ys)
        self.theta = 90.0 + s * (ang - ang[0])  # 左に曲がった形での機体の向き[deg]
        gz = np.array(log["gyro_z"]) if "gyro_z" in log else np.gradient(ang, t)
        self.omega = s * gz  # [dps]
        if "vl" in log and "vr" in log:
            self.v = 0.5 * (np.array(log["vl"]) + np.array(log["vr"]))
        else:
            self.v = np.gradient(np.array(log["dist"]), t)
        # 左に曲がった形では、外 = 右
        self.ad_in = np.array(log["ad_r"] if self.right else log["ad_l"])
        self.ad_out = np.array(log["ad_l"] if self.right else log["ad_r"])
        self.ref_in = params["WALL_REF_R"] if self.right else params["WALL_REF_L"]
        self.ref_out = params["WALL_REF_L"] if self.right else params["WALL_REF_R"]
        self.th_in = params["WALL_TH_R"] if self.right else params["WALL_TH_L"]
        self.th_out = params["WALL_TH_L"] if self.right else params["WALL_TH_R"]
        target = np.array(log["target"]) if "target" in log else self.v
        # 曲がった後、止まるまで(出口の直進)
        after = np.arange(self.i_turn1 + 1, len(t))
        self.win = after[target[after] > 1.0]

        kd = yc.TURN_KINDS[self.kind]
        ex, ey = kd["exit"]
        ux, uy = kd["exit_dir"]
        self.n_out = (uy, -ux)
        self.u_out = (ux, uy)
        self.exit_pt = (ex, ey)

    def _detect_kind(self, om_ref, ang, params):
        m = re.match(r"(s90|l90|l180)[rl]", self.name)
        if m:
            return m.group(1)
        total = abs(ang[-1] - ang[0])
        if total > 135.0:
            return "l180"
        peak = float(np.max(np.abs(om_ref)))
        if abs(peak - params["SLALOM_OMEGA_DPS"]) <= abs(peak - params["FAST_LARGE90_OMEGA_DPS"]):
            return "s90"
        return "l90"

    def kin_lateral(self, idx):
        """車輪とジャイロの軌道の、出口の線からの横のずれ(外が +)"""
        nx, ny = self.n_out
        return (self.x[idx] - self.exit_pt[0]) * nx + (self.y[idx] - self.exit_pt[1]) * ny

    def along_after_turn(self, idx):
        ux, uy = self.u_out
        return (self.x[idx] - self.exit_pt[0]) * ux + (self.y[idx] - self.exit_pt[1]) * uy

    def sensor_lateral(self, idx, slope):
        """横の壁センサーで見た、出口の通路の真ん中からの横のずれ(外が +)。壁がない所は nan。"""
        lat_out = (self.ad_out[idx] - self.ref_out) / slope   # 外の壁に近いほど大きい
        lat_in = -(self.ad_in[idx] - self.ref_in) / slope     # 内の壁に近いほど小さい
        ok_out = self.ad_out[idx] > self.th_out
        ok_in = self.ad_in[idx] > self.th_in
        a = np.where(ok_out, lat_out, np.nan)
        b = np.where(ok_in, lat_in, np.nan)
        both = np.vstack([a, b])
        cnt = np.sum(~np.isnan(both), axis=0)
        sm = np.nansum(both, axis=0)
        return np.where(cnt > 0, sm / np.maximum(cnt, 1), np.nan)

    def slip_lateral_model(self, Ks, C):
        """ログの v と ω でスリップアングルを進めたときの、出口の窓での横のずれ(外が +)。
        Ks: K の配列。戻り値の形は (len(Ks), len(win))。"""
        t, v, om = self.t, self.v, np.radians(self.omega)
        dt = np.diff(t, prepend=t[0])
        u = (v * 1e-3) * om  # β の目標 / K
        filt = np.empty_like(u)
        b = 0.0
        for i in range(len(u)):
            if C <= 0.0:
                b = u[i]
            else:
                b += (1.0 - math.exp(-dt[i] / C)) * (u[i] - b)
            filt[i] = b
        th = np.radians(self.theta)
        beta = np.outer(Ks, filt)  # (nK, nT)
        nx, ny = self.n_out
        # スリップがあるときと無いときの、1歩ごとの進み方の差(横の成分)
        step = (v * dt)[None, :]
        dlat = step * ((np.cos(th - beta) - np.cos(th)) * nx + (np.sin(th - beta) - np.sin(th)) * ny)
        cum = np.cumsum(dlat, axis=1)
        return cum[:, self.win]


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------

class TurnSimApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("yuho turn sim")
        self.font_name = yc.setup_japanese_font()
        self.params = yc.read_params()
        self.logs: list[TurnLog] = []
        self.fit_texts = {}  # 旋回の種類ごとの、合わせた結果
        self.fit_b = {}      # 旋回の種類ごとの、合わせた共通の横のずれ b [mm](右へ)
        self.paste_text = ""

        left = ttk.Frame(root, padding=6)
        left.pack(side=tk.LEFT, fill=tk.Y)
        right = ttk.Frame(root)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # ---- 旋回 ----
        box = ttk.LabelFrame(left, text="旋回", padding=4)
        box.pack(fill=tk.X)
        self.kind = tk.StringVar(value="s90")
        for k, kd in yc.TURN_KINDS.items():
            ttk.Radiobutton(box, text=f"{kd['label']} ({k})", value=k, variable=self.kind,
                            command=self.load_from_params).pack(anchor=tk.W)
        self.right_turn = tk.BooleanVar(value=True)
        row = ttk.Frame(box)
        row.pack(anchor=tk.W)
        ttk.Radiobutton(row, text="右", value=True, variable=self.right_turn, command=self.update).pack(side=tk.LEFT)
        ttk.Radiobutton(row, text="左", value=False, variable=self.right_turn, command=self.update).pack(side=tk.LEFT)

        self.vars = {}
        grid = ttk.Frame(box)
        grid.pack(fill=tk.X, pady=4)
        fields = [
            ("v", "並進の速さ [mm/s]"), ("omega", "最高角速度 [dps]"), ("alpha", "角加速度 [dps²]"),
            ("pre_adj", "PRE_ADJ [mm]"), ("post_adj", "POST_ADJ [mm]"),
            ("K", "スリップ K [s²/m]"), ("C", "スリップ C [s]"),
            ("width", "機体の幅 [mm]"), ("extra", "出口の直進 [mm]"),
        ]
        for r, (key, label) in enumerate(fields):
            ttk.Label(grid, text=label).grid(row=r, column=0, sticky=tk.W)
            var = tk.StringVar()
            ent = ttk.Entry(grid, textvariable=var, width=10)
            ent.grid(row=r, column=1, sticky=tk.W)
            ent.bind("<Return>", lambda e: self.update())
            ent.bind("<FocusOut>", lambda e: self.update())
            self.vars[key] = var
            if key == "omega":
                self.omega_entry = ent
        self.vars["K"].set("0.0")
        self.vars["C"].set("0.0")
        self.vars["width"].set("86")
        self.vars["extra"].set("180")
        brow = ttk.Frame(box)
        brow.pack(fill=tk.X)
        ttk.Button(brow, text="params.h から読む", command=self.reload_params).pack(side=tk.LEFT)
        ttk.Button(brow, text="計算", command=self.update).pack(side=tk.LEFT)

        # ---- 実機のログ ----
        lbox = ttk.LabelFrame(left, text="実機のログ (SLALOM テスト)", padding=4)
        lbox.pack(fill=tk.X, pady=4)
        brow = ttk.Frame(lbox)
        brow.pack(fill=tk.X)
        ttk.Button(brow, text="開く", command=self.open_logs).pack(side=tk.LEFT)
        ttk.Button(brow, text="外す", command=self.remove_log).pack(side=tk.LEFT)
        ttk.Button(brow, text="全部外す", command=self.clear_logs).pack(side=tk.LEFT)
        self.log_list = tk.Listbox(lbox, height=6, width=42, selectmode=tk.EXTENDED, exportselection=False)
        self.log_list.pack(fill=tk.X)
        srow = ttk.Frame(lbox)
        srow.pack(fill=tk.X, pady=2)
        ttk.Label(srow, text="横センサーの傾き [AD/mm]").pack(side=tk.LEFT)
        self.slope = tk.StringVar(value="8.0")
        e = ttk.Entry(srow, textvariable=self.slope, width=6)
        e.pack(side=tk.LEFT)
        e.bind("<Return>", lambda ev: self.update())
        ttk.Button(lbox, text="傾きを探索のログから見積もる", command=self.estimate_slope).pack(fill=tk.X)
        self.fit_bias = tk.BooleanVar(value=True)
        ttk.Checkbutton(lbox, text="右・左で共通の横のずれ b も合わせる\n(WALL_REF・置き方で、どちらに曲がっても\n 同じ側へずれて見える分。右・左の両方のログが要る)",
                        variable=self.fit_bias).pack(anchor=tk.W)
        ttk.Button(lbox, text="K と C をログに合わせる", command=self.fit).pack(fill=tk.X, pady=2)

        # ---- 結果 ----
        rbox = ttk.LabelFrame(left, text="結果", padding=4)
        rbox.pack(fill=tk.BOTH, expand=True)
        self.result = tk.Text(rbox, width=46, height=22, font=("Consolas", 9))
        self.result.pack(fill=tk.BOTH, expand=True)
        ttk.Button(rbox, text="params.h の行をコピー", command=self.copy_paste_text).pack(fill=tk.X)

        # ---- 図 ----
        self.fig = Figure(figsize=(9, 8))
        gs = self.fig.add_gridspec(3, 1)
        self.ax_map = self.fig.add_subplot(gs[0:2, 0])
        self.ax_lat = self.fig.add_subplot(gs[2, 0])
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        NavigationToolbar2Tk(self.canvas, right).update()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.load_from_params()

    # ---- 値 ----
    def fval(self, key, default=0.0):
        try:
            return float(self.vars[key].get())
        except ValueError:
            return default

    def reload_params(self):
        self.params = yc.read_params()
        self.load_from_params()

    def load_from_params(self):
        tp = yc.turn_params_from_h(self.params, self.kind.get())
        self.vars["v"].set(f"{tp['v']:g}")
        self.vars["omega"].set("" if tp["omega"] is None else f"{tp['omega']:g}")
        self.vars["alpha"].set(f"{tp['alpha']:g}")
        self.vars["pre_adj"].set(f"{tp['pre_adj']:g}")
        self.vars["post_adj"].set(f"{tp['post_adj']:g}")
        self.omega_entry.state(["disabled"] if self.kind.get() == "l180" else ["!disabled"])
        self.update()

    # ---- シミュレーション ----
    def simulate(self, K, C, pre_adj, post_adj):
        kind = self.kind.get()
        v, alpha = self.fval("v", 500.0), self.fval("alpha", 8000.0)
        omega, pre0, post0, shape = yc.turn_offsets(kind, v, self.fval("omega", 450.0), alpha)
        sim = yc.simulate_turn(kind, v, omega, alpha, pre0 + pre_adj, post0 + post_adj, K, C,
                               extra_mm=self.fval("extra", 180.0))
        return sim, omega, pre0, post0, shape

    def suggest_adj(self, K, C, pre_adj, post_adj):
        """スリップがあっても出口の線に乗るような PRE/POST_ADJ(前後の直進をずらすだけで直る分)。"""
        kind = self.kind.get()
        for _ in range(3):
            sim = self.simulate(K, C, pre_adj, post_adj)[0]
            e = yc.exit_errors(kind, sim)
            if kind == "l180":
                d_pre = -(e["ymax"] - yc.SECTION_MM)   # 一番奥が 1区画先になるように
                d_post = -(e["along"] - d_pre)         # 前を伸ばすと出口は手前に来る
            else:
                d_pre = -e["lat_final"]                # 前を伸ばすと出口の線が外へずれる
                d_post = -e["along"]
            pre_adj += d_pre
            post_adj += d_post
        return pre_adj, post_adj

    def update(self):
        kind = self.kind.get()
        kd = yc.TURN_KINDS[kind]
        K, C = self.fval("K"), self.fval("C")
        pre_adj, post_adj = self.fval("pre_adj"), self.fval("post_adj")
        sim, omega, pre0, post0, shape = self.simulate(K, C, pre_adj, post_adj)
        ideal = self.simulate(0.0, 0.0, 0.0, 0.0)[0]
        if kind == "l180":
            self.vars["omega"].set(f"{omega:.1f}")
        e = yc.exit_errors(kind, sim)
        dmin, pil = yc.min_pillar_distance(kind, sim["x"], sim["y"])
        half_w = 0.5 * self.fval("width", 86.0)
        sug_pre, sug_post = self.suggest_adj(K, C, pre_adj, post_adj)
        v = self.fval("v", 500.0)
        lat_acc = v * math.radians(omega) / 1000.0  # [m/s²]

        names = yc.ADJ_NAMES[kind]
        self.paste_text = (f"#define {names[0]:<26s}{sug_pre:.1f}f\n"
                           f"#define {names[1]:<26s}{sug_post:.1f}f\n")
        lines = [
            f"{kd['label']} ({'右' if self.right_turn.get() else '左'})",
            f"v {v:.0f} mm/s  ω {omega:.1f} dps  α {self.fval('alpha'):.0f} dps²",
            f"  横の加速度の最大 {lat_acc:.2f} m/s² ({lat_acc / 9.81:.2f} G)",
            f"機体と同じ前後のオフセット: pre {pre0:.2f}  post {post0:.2f} mm",
            f"  + ADJ で実際に走る:       pre {pre0 + pre_adj:.2f}  post {post0 + post_adj:.2f} mm",
            f"曲がる時間 {shape[4] * 1000:.0f} ms、曲がる間の道のり {shape[3]:.1f} mm",
            f"スリップ K={K:g} C={C:g}: β の最大 {max(abs(b) for b in sim['beta']):.2f}°",
            "",
            "出口のずれ (後ろのオフセットの後):",
            f"  前後 {e['along']:+.2f} mm (+ で行き過ぎ)",
            f"  横   {e['lat_post']:+.2f} mm (+ で外)  → 延長後 {e['lat_final']:+.2f} mm",
            f"  進む向き {e['travel_err']:+.2f}° (機体の向きは指令どおり)",
            f"柱との距離 (中心の軌道): {dmin:.1f} mm (柱 {pil[0]:.0f},{pil[1]:.0f})",
            f"  − 幅/2 = {dmin - half_w:.1f} mm{'  ← 当たる' if dmin - half_w < 0 else ''}",
            "  (機体を幅の円とみた目安。角の出っ張りは見ていない)",
            "",
            "ずれを消す調整分 (今の K, C で):",
            self.paste_text.rstrip(),
        ]
        if kind == "l180":
            lines.append(f"  横 {e['lat_final']:+.2f} mm は前後の直進では直せない")
            lines.append("  (機体は ω をスリップなしで求めている)")
        if self.fit_texts.get(kind):
            lines += ["", self.fit_texts[kind]]
        self.result.delete("1.0", tk.END)
        self.result.insert(tk.END, "\n".join(lines))

        self.draw(kind, sim, ideal, half_w)

    # ---- 描く ----
    def draw(self, kind, sim, ideal, half_w):
        mir = -1.0 if self.right_turn.get() else 1.0  # 計算は左に曲がる形、右なら x を反転
        ax = self.ax_map
        ax.clear()
        xs = np.array(sim["x"]) * mir
        ys = np.array(sim["y"])
        all_x = list(xs) + [p * mir for p in yc.TURN_KINDS[kind]["start"][:1]]
        all_y = list(ys) + [yc.TURN_KINDS[kind]["start"][1]]
        for lg in self.logs:
            if lg.kind == kind:
                all_x += list(lg.x * mir)
                all_y += list(lg.y)

        # 区画の線と柱
        for px, py in yc.pillars_near(kind, all_x, all_y, margin=60.0):
            ax.add_patch(Rectangle((px - yc.PILLAR_HALF_MM, py - yc.PILLAR_HALF_MM),
                                   2 * yc.PILLAR_HALF_MM, 2 * yc.PILLAR_HALF_MM, color="0.2"))
        pil = yc.pillars_near(kind, all_x, all_y, margin=60.0)
        for px in sorted({p[0] for p in pil}):
            ax.axvline(px, color="0.85", lw=0.8, zorder=0)
        for py in sorted({p[1] for p in pil}):
            ax.axhline(py, color="0.85", lw=0.8, zorder=0)

        # 機体の幅(向きは機体の向き θ)
        th = np.radians(np.array(sim["theta"]))
        sx_l, sy_l = np.array(sim["x"]), np.array(sim["y"])
        for sgn in (1.0, -1.0):
            ax.plot((sx_l - sgn * half_w * np.sin(th)) * mir, sy_l + sgn * half_w * np.cos(th),
                    color="tab:blue", lw=0.6, alpha=0.4)

        ph = np.array(sim["phase"])
        ax.plot(np.array(ideal["x"]) * mir, ideal["y"], color="0.5", lw=1.0, ls="--", label="スリップなし")
        colors = {0: "tab:green", 1: "tab:red", 2: "tab:green", 3: "tab:olive"}
        for p, c in colors.items():
            m = ph == p
            if m.any():
                ax.plot(xs[m], ys[m], color=c, lw=2.0)
        ax.plot([], [], color="tab:red", lw=2, label="旋回 (緑: 前後のオフセット)")
        ex, ey = yc.TURN_KINDS[kind]["exit"]
        ax.plot([ex * mir], [ey], "k+", ms=12, mew=2, label="出口の目標")

        for lg in self.logs:
            if lg.kind != kind:
                continue
            line, = ax.plot(lg.x * mir, lg.y, lw=1.2, label=f"{lg.name} (車輪+ジャイロ)")
            try:
                slope = float(self.slope.get())
                lat_s = lg.sensor_lateral(lg.win, slope)
                lat_k = lg.kin_lateral(lg.win)
                d = lat_s - lat_k
                nx, ny = lg.n_out
                ax.plot((lg.x[lg.win] + d * nx) * mir, lg.y[lg.win] + d * ny, ".", ms=3,
                        color=line.get_color(), alpha=0.7)
            except ValueError:
                pass
        ax.set_aspect("equal")
        ax.set_xlim(min(min(all_x), min(p[0] for p in pil)) - 20, max(max(all_x), max(p[0] for p in pil)) + 20)
        ax.set_ylim(min(min(all_y), min(p[1] for p in pil)) - 20, max(max(all_y), max(p[1] for p in pil)) + 20)
        ax.set_title("軌道 (点: ログの壁センサーで見た位置)")
        ax.legend(loc="best", fontsize=7)

        # 曲がった後の横のずれ
        ax2 = self.ax_lat
        ax2.clear()
        kd = yc.TURN_KINDS[kind]
        ux, uy = kd["exit_dir"]
        nx, ny = uy, -ux
        sx, sy = np.array(sim["x"]), np.array(sim["y"])
        m = ph >= 2
        along = (sx[m] - ex) * ux + (sy[m] - ey) * uy
        lat = (sx[m] - ex) * nx + (sy[m] - ey) * ny
        ax2.plot(along, lat, color="tab:red", label="シミュレーション")
        K, C = self.fval("K"), self.fval("C")
        try:
            slope = float(self.slope.get())
        except ValueError:
            slope = None
        for lg in self.logs:
            if lg.kind != kind:
                continue
            a = lg.along_after_turn(lg.win)
            l_k = lg.kin_lateral(lg.win)
            line, = ax2.plot(a, l_k, lw=0.8, ls="--")
            if slope:
                l_s = lg.sensor_lateral(lg.win, slope)
                ax2.plot(a, l_s, ".", ms=3, color=line.get_color(), label=lg.name)
                model = lg.slip_lateral_model(np.array([K]), C)[0]
                model = model + self.fit_b.get(kind, 0.0) * (-1.0 if lg.right else 1.0)
                ax2.plot(a, l_k + model, lw=1.0, color=line.get_color())
        ax2.axhline(0.0, color="0.6", lw=0.8)
        ax2.set_xlabel("出口の目標からの前後 [mm]")
        ax2.set_ylabel("横 [mm] (+ が外)")
        ax2.set_title("曲がった後の横のずれ  点: 壁センサー、破線: 車輪+ジャイロ、実線: 車輪+ジャイロ+スリップ(今の K, C、合わせた b)",
                      fontsize=9)
        ax2.legend(loc="best", fontsize=7, ncol=2)
        self.fig.tight_layout()
        self.canvas.draw_idle()

    # ---- ログ ----
    def open_logs(self):
        paths = filedialog.askopenfilenames(
            title="SLALOM テストのログ", initialdir=str(LOG_DIR / "slalom") if (LOG_DIR / "slalom").exists() else str(LOG_DIR),
            filetypes=[("log", "*.csv *.bin"), ("all", "*.*")])
        for p in paths:
            try:
                lg = TurnLog(Path(p), self.params)
            except Exception as ex:  # 読めないログは知らせて飛ばす
                messagebox.showwarning("ログ", f"{Path(p).name}: {ex}")
                continue
            self.logs.append(lg)
            self.log_list.insert(tk.END, f"{lg.name}: {lg.kind} {'右' if lg.right else '左'} "
                                         f"ω {lg.peak_omega:.0f} dps")
        if self.logs and paths:
            last = self.logs[-1]
            if self.kind.get() != last.kind:
                self.kind.set(last.kind)
                self.right_turn.set(last.right)
                self.load_from_params()
                return
            self.right_turn.set(last.right)
        self.update()

    def remove_log(self):
        for i in reversed(self.log_list.curselection()):
            self.log_list.delete(i)
            del self.logs[i]
        self.update()

    def clear_logs(self):
        self.log_list.delete(0, tk.END)
        self.logs.clear()
        self.fit_texts.clear()
        self.fit_b.clear()
        self.update()

    def estimate_slope(self):
        p = filedialog.askopenfilename(
            title="探索のログ (壁の制御をしながら直進している所を使う)",
            initialdir=str(LOG_DIR / "search") if (LOG_DIR / "search").exists() else str(LOG_DIR),
            filetypes=[("log", "*.csv *.bin"), ("all", "*.*")])
        if not p:
            return
        th_l, th_r = self.params.get("WALL_TH_L", 180.0), self.params.get("WALL_TH_R", 150.0)
        k, n, found = yc.estimate_side_slope(yc.load_log(p), th_l, th_r)
        if k is None:
            messagebox.showinfo("傾き", "使える直進が見つからなかった")
            return
        detail = "\n".join(f"  {t:7.2f}s {s} {kk:+6.1f} (r={r:+.2f})" for t, s, kk, r in found)
        messagebox.showinfo("傾き", f"中央値 {k:.1f} AD/mm ({n} 個)\n{detail}\n\n"
                                     "壁に近いほど値の変わり方は大きくなるので、だいたいの値。")
        self.slope.set(f"{k:.1f}")
        self.update()

    def fit(self):
        kind = self.kind.get()
        logs = [lg for lg in self.logs if lg.kind == kind]
        if not logs:
            messagebox.showinfo("合わせる", f"{kind} のログがない")
            return
        try:
            slope = float(self.slope.get())
        except ValueError:
            messagebox.showwarning("合わせる", "傾きが数でない")
            return
        data = []
        for lg in logs:
            meas = lg.sensor_lateral(lg.win, slope) - lg.kin_lateral(lg.win)
            ok = ~np.isnan(meas)
            if ok.sum() >= 5:
                data.append((lg, meas, ok))
        if not data:
            messagebox.showinfo("合わせる", "曲がった後に横の壁が見えている所がない")
            return
        # 測った横のずれ(外が +) = スリップの分 + b × s。b は機体が右へずれて見える分で、
        # 外は左に曲がると右・右に曲がると左なので s = +1(左に曲がった)/ −1(右)。
        use_bias = self.fit_bias.get()
        dirs = {lg.right for lg, _, _ in data}
        n = sum(int(ok.sum()) for _, _, ok in data)
        best = (float("inf"), 0.0, 0.0, 0.0)
        for C in FIT_C:
            models = [(lg.slip_lateral_model(FIT_K, C)[:, ok], meas[ok], -1.0 if lg.right else 1.0)
                      for lg, meas, ok in data]
            b = np.zeros(len(FIT_K))
            if use_bias:
                b = sum(s * np.sum(me[None, :] - mo, axis=1) for mo, me, s in models) / n
            err = np.zeros(len(FIT_K))
            for mo, me, s in models:
                r = mo + (b * s)[:, None] - me[None, :]
                err += np.sum(r * r, axis=1)
            i = int(np.argmin(err))
            if err[i] < best[0]:
                best = (float(err[i]), float(FIT_K[i]), float(C), float(b[i]))
        rms = math.sqrt(best[0] / n)
        # 比べる: スリップなし(b だけ)
        b0 = 0.0
        if use_bias:
            b0 = sum((-1.0 if lg.right else 1.0) * float(np.sum(meas[ok])) for lg, meas, ok in data) / n
        rms0 = math.sqrt(sum(float(np.sum((b0 * (-1.0 if lg.right else 1.0) - meas[ok]) ** 2))
                             for lg, meas, ok in data) / n)
        self.fit_b[kind] = best[3]
        self.vars["K"].set(f"{best[1]:.4f}")
        self.vars["C"].set(f"{best[2]:.3f}")
        notes = []
        if best[1] >= FIT_K[-1] - 1e-9:
            notes.append("K が探す範囲の端")
        if best[2] >= FIT_C[-1] - 1e-9:
            notes.append("C が探す範囲の端")
        if use_bias and len(dirs) < 2:
            notes.append("片方の向きのログだけなので b と K を分けられない")
        self.fit_texts[kind] = (f"ログに合わせた結果 ({len(data)} 本, {n} 点):\n"
                         f"  K = {best[1]:.4f}, C = {best[2]:.3f}"
                         + (f", b = {best[3]:+.1f} mm (右へ)" if use_bias else "") + "\n"
                         f"  残りのずれ RMS {rms:.2f} mm (スリップなしなら {rms0:.2f} mm)"
                         + "".join(f"\n  ※ {s}" for s in notes)
                         + f"\n  (傾き {slope:g} AD/mm を使った。傾きが違えば K も変わる)")
        self.update()

    def copy_paste_text(self):
        self.root.clipboard_clear()
        self.root.clipboard_append(self.paste_text)


def main():
    root = tk.Tk()
    root.geometry("1400x900")
    TurnSimApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
