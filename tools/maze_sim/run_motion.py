"""最短走行の指令(logic/maze/run_path.h)を、表示用の固定の形にする(GUI の描画と再生用)。

形は速度のパラメータによらない理想的な形で、実機の軌道(スラロームの形)とは一致しない。
  - 座標は mm。区画 (x, y) の中心が ((x+0.5)S, (y+0.5)S)、x が東、y が北(S = SECTION_MM)。
  - 直進: 線分(半区画単位)。
  - 小回り: 境目から境目まで、半径 S/2 の 90° の円弧。
  - 大回り90°: 区画の中心から中心まで、半径 S の円弧。
  - 大回り180°: 区画の中心から隣の列の区画の中心まで、半区画直進 + 柱を回る半径 S/2 の半円
    + 半区画直進。
再生の速さ: 直進は台形加速(入りと出は前後の旋回の速度、列の端なら 0)、旋回は一定の速さ。
指令ごとの時間は C が求めた値(MazeSim.run_plan() の times)に合わせる。
"""

import math

import sim_lib as sl

DIR_VEC = [(0.0, 1.0), (1.0, 0.0), (0.0, -1.0), (-1.0, 0.0)]  # 北 東 南 西


class Line:
    def __init__(self, p0, d, length):
        self.p0 = p0
        self.u = DIR_VEC[d]
        self.length = length
        self.heading = math.atan2(self.u[1], self.u[0])

    def at(self, s):
        """始点から s [mm] の位置と向き(数学の角度 [rad]、東が 0、反時計回りが正)"""
        return (self.p0[0] + self.u[0] * s, self.p0[1] + self.u[1] * s), self.heading


class Arc:
    def __init__(self, p0, d, q, radius, quarters):
        """p0 から向き d で入り、q(右 +1 / 左 -1)へ quarters × 90° 曲がる円弧"""
        vx, vy = DIR_VEC[(d + q) % 4]
        self.c = (p0[0] + vx * radius, p0[1] + vy * radius)
        self.r = radius
        self.a0 = math.atan2(p0[1] - self.c[1], p0[0] - self.c[0])
        self.sign = -q  # 右旋回は時計回り(角度が減る)
        self.length = radius * math.pi / 2 * quarters

    def at(self, s):
        a = self.a0 + self.sign * s / self.r
        pos = (self.c[0] + self.r * math.cos(a), self.c[1] + self.r * math.sin(a))
        return pos, a + self.sign * math.pi / 2


class Chain:
    """線分と円弧をつないだ形(大回り180°)"""

    def __init__(self, pieces):
        self.pieces = pieces
        self.length = sum(p.length for p in pieces)

    def at(self, s):
        for p in self.pieces:
            if s <= p.length or p is self.pieces[-1]:
                return p.at(min(s, p.length))
            s -= p.length


def trapezoid(length, v_in, v_out, accel, vmax):
    """台形加速の (時間 [s], 時刻 t → (距離, 速度) の関数)。加速度が足りなければ一様に変える"""
    if length <= 0:
        return 0.0, lambda t: (0.0, v_in)
    if abs(v_out * v_out - v_in * v_in) > 2 * accel * length:
        a = (v_out * v_out - v_in * v_in) / (2 * length)
        return 2 * length / (v_in + v_out), lambda t: (v_in * t + 0.5 * a * t * t, v_in + a * t)
    vp = math.sqrt((2 * accel * length + v_in * v_in + v_out * v_out) / 2)
    vp = max(min(vp, vmax), v_in, v_out)
    t_acc = (vp - v_in) / accel
    t_dec = (vp - v_out) / accel
    d_acc = (vp * vp - v_in * v_in) / (2 * accel)
    d_dec = (vp * vp - v_out * v_out) / (2 * accel)
    t_cruise = max(0.0, length - d_acc - d_dec) / vp

    def f(t):
        if t < t_acc:
            return v_in * t + 0.5 * accel * t * t, v_in + accel * t
        t2 = t - t_acc
        if t2 < t_cruise:
            return d_acc + vp * t2, vp
        t3 = min(t2 - t_cruise, t_dec)
        return d_acc + vp * t_cruise + vp * t3 - 0.5 * accel * t3 * t3, vp - accel * t3
    return t_acc + t_cruise + t_dec, f


class Segment:
    """指令1つ分の形と時間。motion(t) は時刻 → (距離, 速度)、own_time はその動きの時間"""

    def __init__(self, cmd_type, piece, duration, own_time, motion):
        self.type = cmd_type
        self.piece = piece
        self.duration = duration  # C が求めた時間(再生はこれに合わせる)
        self._own = own_time
        self._motion = motion

    def state_at(self, t):
        """指令の始めから t [s] 後の (位置, 向き, 速度)"""
        t = min(max(t, 0.0), self.duration)
        # C の時間と少し違っても形の終わりで止まるように、時刻を伸び縮みさせる
        scale = self._own / self.duration if self.duration > 0 else 0.0
        s, v = self._motion(t * scale)
        pos, heading = self.piece.at(min(max(s, 0.0), self.piece.length))
        return pos, heading, v

    def points(self, step_mm=10.0):
        """描画用の点列"""
        n = max(2, int(self.piece.length / step_mm) + 1)
        return [self.piece.at(self.piece.length * i / (n - 1))[0] for i in range(n)]


class RunMotion:
    """指令の列全体の形と時間。時刻 → 位置・向き を求める。"""

    def __init__(self, cmds, times, prof, start_cell):
        """prof は MazeSim.run_profile()"""
        S = prof["section"]
        pos = ((start_cell[0] + 0.5) * S, (start_cell[1] + 0.5) * S)
        d = 0  # スタートは北向き
        self.segments = []

        def turn_speed(i):
            if 0 <= i < len(cmds) and sl.RUN_SMALL90_R <= cmds[i][0] <= sl.RUN_LARGE180_L:
                return prof["v_turn"][cmds[i][0]]
            return 0.0

        for i, ((t, halves), dt) in enumerate(zip(cmds, times)):
            if t == sl.RUN_STOP:
                break
            if t == sl.RUN_STRAIGHT:
                piece = Line(pos, d, halves * S / 2)
                own, motion = trapezoid(piece.length, turn_speed(i - 1), turn_speed(i + 1),
                                        prof["accel"], prof["vmax"])
            else:
                q = 1 if sl.RUN_QUARTER_TURNS[t] > 0 else -1
                if t in (sl.RUN_SMALL90_R, sl.RUN_SMALL90_L):
                    piece = Arc(pos, d, q, S / 2, 1)
                elif t in (sl.RUN_LARGE90_R, sl.RUN_LARGE90_L):
                    piece = Arc(pos, d, q, S, 1)
                else:
                    a = Line(pos, d, S / 2)
                    b = Arc(a.at(a.length)[0], d, q, S / 2, 2)
                    piece = Chain([a, b, Line(b.at(b.length)[0], (d + 2) % 4, S / 2)])
                v = prof["v_turn"][t]
                own, motion = piece.length / v, (lambda tt, v=v: (v * tt, v))
            self.segments.append(Segment(t, piece, dt, own, motion))
            pos = piece.at(piece.length)[0]
            d = (d + sl.RUN_QUARTER_TURNS[t]) % 4

        self.starts = []
        total = 0.0
        for seg in self.segments:
            self.starts.append(total)
            total += seg.duration
        self.duration = total

    def state_at(self, t):
        """時刻 t [s] の (位置 (x, y) [mm], 向き [rad], 速度 [mm/s], 指令の番号)"""
        if not self.segments:
            return None
        t = min(max(t, 0.0), self.duration)
        i = len(self.segments) - 1
        for k, t0 in enumerate(self.starts):
            if t < t0 + self.segments[k].duration:
                i = k
                break
        pos, heading, v = self.segments[i].state_at(t - self.starts[i])
        return pos, heading, v, i
