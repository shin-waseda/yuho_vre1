#!/usr/bin/env python3
"""yuho の PC ツール(turn_sim.py / log_viewer.py)で共通に使う部品。

- read_params(): Core/Inc/params.h の #define を読み、数の値にして返す(他の名前を使った式も計算する)。
- VelocityProfile: 機体の logic/control/velocity_profile.c と同じ台形(同じ式・同じ刻み)。
- load_log(): get_log.py が作った CSV(または SD の .bin)を読み、列名 → 値の配列 の辞書にする。
- reconstruct_path(): ログの dist(左右の車輪の平均の距離)と angle(ジャイロの向き)から走った軌道を作る。
"""

import csv
import math
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PARAMS_H = ROOT / "Core" / "Inc" / "params.h"

SECTION_MM = 180.0
CONTROL_DT_S = 0.001
PILLAR_HALF_MM = 6.0  # 柱は 12mm 角


# ---------------------------------------------------------------------------
# params.h
# ---------------------------------------------------------------------------

def read_params(path: Path = PARAMS_H) -> dict:
    """#define NAME 式 を読み、計算できたものを {NAME: float} で返す。"""
    raw = {}
    pat = re.compile(r"^\s*#define\s+([A-Za-z_]\w*)\s+(.+?)\s*(?://.*)?$")
    for line in path.read_text(encoding="utf-8").splitlines():
        m = pat.match(line)
        if m:
            raw[m.group(1)] = m.group(2)

    values = {}

    def clean(expr: str) -> str:
        expr = re.sub(r"\(\s*(?:float|uint32_t|int|uint16_t|uint8_t|int16_t)\s*\)", "", expr)
        expr = re.sub(r"(\d+\.?\d*(?:[eE][-+]?\d+)?)[fFuUlL]+\b", r"\1", expr)
        return expr

    def evaluate(name, depth=0):
        if name in values:
            return values[name]
        if name not in raw or depth > 20:
            raise KeyError(name)
        expr = clean(raw[name])

        def repl(m):
            return repr(evaluate(m.group(0), depth + 1))

        expr2 = re.sub(r"\b[A-Za-z_]\w*\b", repl, expr)
        val = float(eval(expr2, {"__builtins__": {}}, {}))
        values[name] = val
        return val

    for name in raw:
        try:
            evaluate(name)
        except Exception:
            pass  # 数でない定義(配列など)は飛ばす
    return values


# ---------------------------------------------------------------------------
# 機体と同じ台形(logic/control/velocity_profile.c)
# ---------------------------------------------------------------------------

PROFILE_DECEL_MARGIN = 1.5


class VelocityProfile:
    """機体の VelocityProfile_Start / VelocityProfile_Step と同じ計算。"""

    def __init__(self, distance, v_start, v_max, v_end, accel):
        self.distance = distance
        self.v_max = v_max
        self.v_end = min(v_end, v_max)
        self.accel = accel
        self.pos = 0.0
        self.v = v_start
        self.a = 0.0
        self.decelerating = False
        self.done = distance <= 0.0 or accel <= 0.0

    def step(self, dt=CONTROL_DT_S):
        if self.done:
            self.v = self.v_end
            self.a = 0.0
            return
        remaining = self.distance - self.pos
        if not self.decelerating:
            decel_dist = 0.0
            if self.v > self.v_end:
                decel_dist = (self.v * self.v - self.v_end * self.v_end) / (2.0 * self.accel)
            if remaining - self.v * dt <= decel_dist:
                self.decelerating = True
        if self.decelerating:
            if remaining > 1e-6 and self.v > self.v_end:
                a_cmd = -(self.v * self.v - self.v_end * self.v_end) / (2.0 * remaining)
                a_cmd = max(a_cmd, -PROFILE_DECEL_MARGIN * self.accel)
            else:
                a_cmd = 0.0
        elif self.v < self.v_max:
            a_cmd = self.accel
        else:
            a_cmd = 0.0
        v_next = self.v + a_cmd * dt
        if a_cmd > 0.0 and v_next > self.v_max:
            v_next = self.v_max
        if a_cmd < 0.0 and v_next < self.v_end:
            v_next = self.v_end
        if v_next < 0.0:
            v_next = 0.0
        self.pos += 0.5 * (self.v + v_next) * dt
        self.a = (v_next - self.v) / dt
        self.v = v_next
        reached = self.pos >= self.distance - 1e-3
        slowed = self.decelerating and self.v <= self.v_end and (self.v_end > 0.0 or self.v <= 0.0)
        if reached or slowed:
            self.done = True
            self.v = self.v_end
            self.a = 0.0


# ---------------------------------------------------------------------------
# ログ
# ---------------------------------------------------------------------------

def load_log(path) -> dict:
    """CSV(get_log.py が作ったもの)か .bin を読み、{列名: [値...]} を返す。ev_text は文字のまま。"""
    path = Path(path)
    if path.suffix.lower() == ".bin":
        import get_log  # tools/get_log.py
        names, rows = get_log.parse_ylog(path.read_bytes())
        cols = {n: [r[i] for r in rows] for i, n in enumerate(names)}
        texts = get_log.event_texts(names, rows)
        if texts is not None:
            cols["ev_text"] = texts
        return cols
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        names = next(reader)
        cols = {n: [] for n in names}
        for row in reader:
            if not row:
                continue
            for n, v in zip(names, row):
                if n == "ev_text":
                    cols[n].append(v)
                else:
                    try:
                        cols[n].append(float(v))
                    except ValueError:
                        cols[n].append(float("nan"))
    return cols


def reconstruct_path(log: dict, x0=0.0, y0=0.0, heading0_deg=90.0, i0=0, i1=None):
    """ログの dist と angle から、i0〜i1 の行の軌道 (xs, ys) を作る(x 右・y 上、heading0 は i0 の向き)。
    dist が急に減った所(制御を有効にし直して 0 に戻った所)は、進んでいないものとして扱う。
    車輪とジャイロだけで作るので、横滑りは見えない(進む向き = 機体の向き としている)。"""
    dist = log["dist"]
    ang = log["angle"]
    if i1 is None:
        i1 = len(dist)
    a0 = ang[i0]
    xs, ys = [x0], [y0]
    x, y = x0, y0
    for i in range(i0 + 1, i1):
        ds = dist[i] - dist[i - 1]
        if ds < -5.0:  # 有効にし直して 0 に戻った
            ds = 0.0
        th = math.radians(heading0_deg + (ang[i] - a0))
        x += ds * math.cos(th)
        y += ds * math.sin(th)
        xs.append(x)
        ys.append(y)
    return xs, ys


def find_events(log: dict, name: str = None):
    """ev_text のある行を [(行の番号, 時刻, 文字), ...] で返す(name を付けたら、その名前のものだけ)。"""
    texts = log.get("ev_text")
    if texts is None:
        return []
    t = log["time_s"]
    out = []
    for i, s in enumerate(texts):
        if s and (name is None or s.split(" ", 1)[0] == name):
            out.append((i, t[i], s))
    return out


def event_values(text: str) -> dict:
    """'STEP x=1 y=0 heading=0 ...' → {'x': 1.0, 'y': 0.0, ...}"""
    vals = {}
    for tok in text.split()[1:]:
        if "=" in tok:
            k, v = tok.split("=", 1)
            try:
                vals[k] = float(v)
            except ValueError:
                pass
    return vals


# ---------------------------------------------------------------------------
# 日本語の文字(matplotlib)
# ---------------------------------------------------------------------------

def setup_japanese_font():
    """入っている日本語のフォントを探して matplotlib に使わせる(決まったパスは使わない)。
    見つからなければ何もしない(日本語が □ になるだけで、動きはする)。"""
    import matplotlib
    from matplotlib import font_manager
    candidates = ["Yu Gothic", "Meiryo", "MS Gothic", "BIZ UDGothic", "Noto Sans CJK JP",
                  "Noto Sans JP", "IPAexGothic", "IPAGothic", "Hiragino Sans", "TakaoGothic"]
    have = {f.name for f in font_manager.fontManager.ttflist}
    for name in candidates:
        if name in have:
            matplotlib.rcParams["font.family"] = [name, "DejaVu Sans"]
            matplotlib.rcParams["axes.unicode_minus"] = False
            return name
    return None


# ---------------------------------------------------------------------------
# スラローム(logic/control/slalom.c と同じ計算)+ スリップアングル
# ---------------------------------------------------------------------------

def slalom_shape(v, omega, alpha, angle):
    """Slalom_ComputeShape と同じ。(forward, side, forward_max, length, time) を返す。"""
    prof = VelocityProfile(angle, 0.0, omega, 0.0, alpha)
    theta = 0.0
    fwd = side = fmax = 0.0
    ticks = 0
    while not prof.done and ticks < 100000:
        prof.step(CONTROL_DT_S)
        theta += prof.v * CONTROL_DT_S
        th = math.radians(theta)
        fwd += v * math.cos(th) * CONTROL_DT_S
        side += v * math.sin(th) * CONTROL_DT_S
        fmax = max(fmax, fwd)
        ticks += 1
    t = ticks * CONTROL_DT_S
    return fwd, side, fmax, v * t, t


def slalom_solve_omega_for_side(v, alpha, angle, side_mm):
    """Slalom_SolveOmegaForSide と同じ(二分法 30 回)。"""
    lo, hi = 30.0, 3000.0
    for _ in range(30):
        om = 0.5 * (lo + hi)
        if slalom_shape(v, om, alpha, angle)[1] > side_mm:
            lo = om
        else:
            hi = om
    return 0.5 * (lo + hi)


def slip_step(beta, v_mm_s, omega_rad_s, K, C, dt):
    """スリップアングル β[rad] を1歩進める。目標は K × v[m/s] × ω[rad/s](1次遅れ、時定数 C[s])。
    C = 0 なら遅れなし(β = 目標)。β が + なら、進む向きは機体の向きより曲がる側の反対(外)へずれる。"""
    target = K * (v_mm_s * 1e-3) * omega_rad_s
    if C <= 0.0:
        return target
    return beta + (1.0 - math.exp(-dt / C)) * (target - beta)


# 旋回の種類ごとの形(左に曲がるとして、入る向きは +y。右は x を反転して描く)。
#   entry: 前のオフセットを測り始める点、exit: 後ろのオフセットの後に着くはずの点、exit_dir: 出る向き
#   start: slalom_test で尻当ての後に記録を始める位置(区画の真ん中)
#   pillar_y0: 柱の y の並び(x は 90 + 180i)
TURN_KINDS = {
    "s90": dict(label="小回り 90°", angle=90.0, entry=(0.0, 0.0), exit=(-90.0, 90.0),
                exit_dir=(-1.0, 0.0), start=(0.0, -270.0), pillar_y0=0.0),
    "l90": dict(label="大回り 90°", angle=90.0, entry=(0.0, 0.0), exit=(-180.0, 180.0),
                exit_dir=(-1.0, 0.0), start=(0.0, -180.0), pillar_y0=90.0),
    "l180": dict(label="大回り 180°", angle=180.0, entry=(0.0, 0.0), exit=(-180.0, 0.0),
                 exit_dir=(0.0, -1.0), start=(0.0, -180.0), pillar_y0=90.0),
}


def turn_params_from_h(p: dict, kind: str) -> dict:
    """params.h の値から、機体と同じ旋回の値(v, ω, α, 調整分)を作る(l180 の ω は None = 求める)。"""
    if kind == "s90":
        v, om, al = p["SLALOM_V_MM_S"], p["SLALOM_OMEGA_DPS"], p["SLALOM_ALPHA_DPS2"]
        pre_adj, post_adj = p.get("SLALOM_PRE_ADJ_MM", 0.0), p.get("SLALOM_POST_ADJ_MM", 0.0)
    elif kind == "l90":
        v, om, al = p["FAST_LARGE90_V_MM_S"], p["FAST_LARGE90_OMEGA_DPS"], p["FAST_LARGE90_ALPHA_DPS2"]
        pre_adj, post_adj = p.get("FAST_LARGE90_PRE_ADJ_MM", 0.0), p.get("FAST_LARGE90_POST_ADJ_MM", 0.0)
    else:
        v, al = p["FAST_LARGE180_V_MM_S"], p["FAST_LARGE180_ALPHA_DPS2"]
        om = None
        pre_adj, post_adj = p.get("FAST_LARGE180_PRE_ADJ_MM", 0.0), p.get("FAST_LARGE180_POST_ADJ_MM", 0.0)
    return dict(v=v, omega=om, alpha=al, pre_adj=pre_adj, post_adj=post_adj)


ADJ_NAMES = {
    "s90": ("SLALOM_PRE_ADJ_MM", "SLALOM_POST_ADJ_MM"),
    "l90": ("FAST_LARGE90_PRE_ADJ_MM", "FAST_LARGE90_POST_ADJ_MM"),
    "l180": ("FAST_LARGE180_PRE_ADJ_MM", "FAST_LARGE180_POST_ADJ_MM"),
}


def turn_offsets(kind, v, omega, alpha):
    """機体と同じ前後のオフセット(調整分を足す前)。l180 は omega を求め直す。(omega, pre, post, shape)"""
    angle = TURN_KINDS[kind]["angle"]
    if kind == "l180":
        omega = slalom_solve_omega_for_side(v, alpha, angle, SECTION_MM)
        sh = slalom_shape(v, omega, alpha, angle)
        pre = SECTION_MM - sh[2]
        post = pre + sh[0]
    else:
        sh = slalom_shape(v, omega, alpha, angle)
        span = SECTION_MM * (0.5 if kind == "s90" else 1.0)
        pre = span - sh[0]
        post = span - sh[1]
    return omega, pre, post, sh


def simulate_turn(kind, v, omega, alpha, pre, post, K=0.0, C=0.0, extra_mm=SECTION_MM):
    """左に曲がるとして、entry から 前のオフセット → 旋回 → 後ろのオフセット → extra_mm 直進 を 1ms 刻みで進める。
    機体の向き θ は機体と同じ台形で回し、位置は θ − β の向きに v で進める(β はスリップアングル)。
    戻り値: dict(x, y, theta[deg], beta[deg], phase(0 前, 1 旋回, 2 後ろ, 3 延長), i_post_end)"""
    kd = TURN_KINDS[kind]
    dt = CONTROL_DT_S
    st = dict(x=kd["entry"][0], y=kd["entry"][1], th=90.0, beta=0.0)
    out = dict(x=[st["x"]], y=[st["y"]], theta=[90.0], beta=[0.0], phase=[0])

    def tick(om, phase, h=dt):
        st["beta"] = slip_step(st["beta"], v, math.radians(om), K, C, h)
        st["th"] += om * h
        a = math.radians(st["th"]) - st["beta"]
        st["x"] += v * h * math.cos(a)
        st["y"] += v * h * math.sin(a)
        out["x"].append(st["x"]); out["y"].append(st["y"]); out["theta"].append(st["th"])
        out["beta"].append(math.degrees(st["beta"])); out["phase"].append(phase)

    def straight(dist, phase):
        # 端の誤差が出ないよう、最後の1歩は残りの距離だけ進む
        n_full = int(max(0.0, dist) // (v * dt))
        for _ in range(n_full):
            tick(0.0, phase)
        rest = max(0.0, dist) - n_full * v * dt
        if rest > 1e-9:
            tick(0.0, phase, rest / v)

    straight(pre, 0)
    prof = VelocityProfile(kd["angle"], 0.0, omega, 0.0, alpha)
    while not prof.done:
        prof.step(dt)
        tick(prof.v, 1)
    straight(post, 2)
    out["i_post_end"] = len(out["x"]) - 1
    straight(extra_mm, 3)
    return out


def exit_errors(kind, sim):
    """後ろのオフセットの後のずれ。外向き(曲がる中心の反対)を + とした横のずれと、出る向きの前後のずれ。
    lat_post: 後ろのオフセットの後、lat_final: 延長の直進の後(スリップが収まった後)。"""
    kd = TURN_KINDS[kind]
    ex, ey = kd["exit"]
    ux, uy = kd["exit_dir"]
    nx, ny = uy, -ux  # 左に曲がったとき、出る向きの右 = 外
    i = sim["i_post_end"]
    dx, dy = sim["x"][i] - ex, sim["y"][i] - ey
    dxf, dyf = sim["x"][-1] - ex, sim["y"][-1] - ey
    return dict(along=dx * ux + dy * uy, lat_post=dx * nx + dy * ny, lat_final=dxf * nx + dyf * ny,
                travel_err=sim["theta"][i] - sim["beta"][i] - (90.0 + kd["angle"]),
                ymax=max(sim["y"]))


def pillars_near(kind, xs, ys, margin=200.0):
    """軌道の近くの柱の中心。"""
    y0 = TURN_KINDS[kind]["pillar_y0"]
    i0 = int(math.floor((min(xs) - margin - 90.0) / SECTION_MM))
    i1 = int(math.ceil((max(xs) + margin - 90.0) / SECTION_MM))
    j0 = int(math.floor((min(ys) - margin - y0) / SECTION_MM))
    j1 = int(math.ceil((max(ys) + margin - y0) / SECTION_MM))
    return [(90.0 + SECTION_MM * i, y0 + SECTION_MM * j)
            for i in range(i0, i1 + 1) for j in range(j0, j1 + 1)]


def min_pillar_distance(kind, xs, ys):
    """機体の中心の軌道と柱(12mm 角)の縁との一番近い距離と、その柱。"""
    best = (float("inf"), None)
    for px, py in pillars_near(kind, xs, ys, margin=50.0):
        for x, y in zip(xs[::2], ys[::2]):
            dx = max(abs(x - px) - PILLAR_HALF_MM, 0.0)
            dy = max(abs(y - py) - PILLAR_HALF_MM, 0.0)
            d = math.hypot(dx, dy)
            if d < best[0]:
                best = (d, (px, py))
    return best


# ---------------------------------------------------------------------------
# 横の壁センサー(L, R)の傾き [AD/mm] を探索のログから見積もる
# ---------------------------------------------------------------------------

def estimate_side_slope(log: dict, th_l=180.0, th_r=150.0, min_range_mm=3.0, min_r=0.6):
    """探索のログの直進(omega_ref = 0、target > 200)で、車輪とジャイロから求めた横の動きと L / R の値の
    回帰の傾きを取る。左へ動くと L は増え R は減るはず(その向きになったものだけ使う)。
    戻り値: (中央値の |傾き| or None, 使った数, [(時刻, 側, 傾き, r), ...])"""
    import numpy as np
    need = ("omega_ref", "target", "angle", "angle_ref", "wall_ofs", "dist", "ad_l", "ad_r")
    if any(k not in log for k in need):
        return None, 0, []
    t = np.array(log["time_s"]); om = np.array(log["omega_ref"]); v = np.array(log["target"])
    ang = np.array(log["angle"]); ar = np.array(log["angle_ref"]); wo = np.array(log["wall_ofs"])
    d = np.array(log["dist"]); al = np.array(log["ad_l"]); arr = np.array(log["ad_r"])
    idx = np.where((np.abs(om) < 1.0) & (v > 200.0))[0]
    if len(idx) == 0:
        return None, 0, []
    found = []
    for r in np.split(idx, np.where(np.diff(idx) != 1)[0] + 1):
        if len(r) < 30:
            continue
        axis = ar[r] - wo[r]
        ds = np.diff(d[r], prepend=d[r][0])
        ylat = np.cumsum(ds * np.sin(np.radians(ang[r] - axis)))  # 左が +
        for side, a, th, sign in (("L", al[r], th_l, 1.0), ("R", arr[r], th_r, -1.0)):
            ok = a > th
            if ok.sum() < 20 or np.ptp(ylat[ok]) < min_range_mm:
                continue
            k = np.polyfit(ylat[ok], a[ok], 1)[0]
            cc = np.corrcoef(ylat[ok], a[ok])[0, 1]
            if sign * cc >= min_r and sign * k > 0:
                found.append((float(t[r[0]]), side, float(k), float(cc)))
    if not found:
        return None, 0, []
    ks = sorted(abs(f[2]) for f in found)
    return ks[len(ks) // 2], len(found), found
