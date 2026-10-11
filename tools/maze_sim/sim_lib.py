"""yuho の迷路 logic 層を、共有ライブラリ (sim_api.c) 経由で Python から使う。

ファームウェアと同じソース (Core/Src/logic/...) と params.h から作るので、
logic 層や params.h を書き換えると、次に読み込むときに自動でビルドし直す。
(ソースとヘッダの中身のハッシュを build/.lib_hash に覚えておき、変わっていたらビルドする)
"""

import ctypes
import glob
import hashlib
import os
import platform
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
BUILD_DIR = os.path.join(HERE, "build")
HASH_FILE = os.path.join(BUILD_DIR, ".lib_hash")

# sim_step() の戻り値 (sim_api.c と同じ)
SIM_MOVED, SIM_AT_GOAL, SIM_DONE, SIM_FAILED, SIM_CRASH, SIM_LOST = range(6)
STATUS_NAMES = ["moving", "at goal", "done", "FAILED", "CRASH", "LOST"]

# sim_phase() の戻り値 (SearchPhase と同じ)
PHASE_NAMES = ["to goal", "to start", "done", "failed", "full search"]
PHASE_TO_GOAL, PHASE_TO_START, PHASE_DONE, PHASE_FAILED, PHASE_FULL = range(5)

# 探索の行き先 (search_time.h の SearchTimeScope と同じ番号)
SCOPE_ROUND, SCOPE_ONE_WAY, SCOPE_FULL = 1, 2, 3
SCOPE_NAMES = {SCOPE_ROUND: "round", SCOPE_ONE_WAY: "oneway", SCOPE_FULL: "full"}

# ActionType (logic/command.h と同じ)
ACTION_STOP, ACTION_FORWARD, ACTION_TURN_RIGHT, ACTION_TURN_LEFT, ACTION_TURN_BACK = range(5)
ACTION_NAMES = ["STOP", "FORWARD", "RIGHT", "LEFT", "BACK"]
ACTION_QUARTER_TURNS = [0, 0, 1, -1, 2]

# RunType (logic/maze/run_path.h と同じ)
(RUN_STOP, RUN_STRAIGHT, RUN_SMALL90_R, RUN_SMALL90_L,
 RUN_LARGE90_R, RUN_LARGE90_L, RUN_LARGE180_R, RUN_LARGE180_L) = range(8)
RUN_NAMES = ["STOP", "STRAIGHT", "SMALL90_R", "SMALL90_L", "LARGE90_R", "LARGE90_L", "LARGE180_R", "LARGE180_L"]
RUN_QUARTER_TURNS = [0, 0, 1, -1, 1, -1, 2, -2]

# sim_run_plan() の kind (sim_api.c と同じ)
PLAN_TIME, PLAN_COST, PLAN_TIME_BEST = range(3)
PLAN_NAMES = ["time-optimal", "cost route + large turns", "best possible (all walls)"]

# sim_wall() の戻り値
WALL_OPEN, WALL_EXISTS, WALL_UNKNOWN = range(3)

# 向き (Direction と同じ: 北0 東1 南2 西3)
DIR_DX = [0, 1, 0, -1]
DIR_DY = [1, 0, -1, 0]

ALGO_DIJKSTRA, ALGO_ADACHI = 0, 1
ALGO_NAMES = ["dijkstra", "adachi"]


def _library_path():
    system = platform.system().lower()
    if system == "windows":
        return os.path.join(BUILD_DIR, "maze_sim_lib.dll")
    if system == "darwin":
        return os.path.join(BUILD_DIR, "libmaze_sim.dylib")
    return os.path.join(BUILD_DIR, "libmaze_sim.so")


def _sources():
    srcs = [os.path.join(HERE, "sim_api.c"), os.path.join(HERE, "sim_core.c"), os.path.join(HERE, "search_time.c"),
            os.path.join(HERE, "plant_bridge.c"), os.path.join(ROOT, "Core", "Src", "logic", "command.c")]
    srcs += sorted(glob.glob(os.path.join(ROOT, "Core", "Src", "logic", "maze", "*.c")))
    # 最短走行の旋回の表(RunProfile_ForSpeeds)と、スラロームのずれのモデル
    srcs += [os.path.join(ROOT, "Core", "Src", "logic", "control", n) for n in ("slalom.c", "velocity_profile.c")]
    return srcs


def _hashed_files():
    """ビルド結果に影響するファイル(ソースと、それが読むヘッダ)"""
    files = _sources()
    files += [os.path.join(HERE, "sim_core.h"), os.path.join(HERE, "search_time.h"), os.path.join(HERE, "plant_bridge.h"),
              os.path.join(ROOT, "Core", "Inc", "params.h"),
              os.path.join(ROOT, "Core", "Inc", "global.h"),
              os.path.join(ROOT, "Core", "Inc", "logic", "command.h")]
    files += sorted(glob.glob(os.path.join(ROOT, "Core", "Inc", "logic", "maze", "*.h")))
    files += sorted(glob.glob(os.path.join(ROOT, "Core", "Inc", "logic", "control", "*.h")))
    return files


def _current_hash():
    h = hashlib.sha256()
    for path in _hashed_files():
        h.update(os.path.relpath(path, ROOT).replace("\\", "/").encode("utf-8"))
        with open(path, "rb") as f:
            h.update(f.read())
    return h.hexdigest()


def build_library(force=False):
    """必要なら共有ライブラリをビルドし、そのパスを返す。"""
    lib = _library_path()
    os.makedirs(BUILD_DIR, exist_ok=True)
    current = _current_hash()
    old = None
    if os.path.exists(HASH_FILE):
        with open(HASH_FILE, encoding="utf-8") as f:
            old = f.read().strip()
    if not force and os.path.exists(lib) and old == current:
        return lib

    flags = ["-std=c11", "-Wall", "-Wextra", "-O2", "-shared", "-I" + os.path.join(ROOT, "Core", "Inc")]
    if platform.system().lower() == "windows":
        flags.append("-static-libgcc")  # Python から読むとき libgcc の DLL を探さずに済むように
    else:
        flags.append("-fPIC")
    cmd = ["gcc"] + flags + ["-o", lib] + _sources() + ["-lm"]
    print("[build]", " ".join(os.path.relpath(c, ROOT) if os.path.isabs(c) else c for c in cmd))
    subprocess.check_call(cmd)

    with open(HASH_FILE, "w", encoding="utf-8") as f:
        f.write(current)
    return lib


class MazeSim:
    """sim_api.c の関数を Python から呼びやすくしたもの。"""

    def __init__(self):
        self.dll = ctypes.CDLL(build_library())
        u8p = ctypes.POINTER(ctypes.c_uint8)
        u16p = ctypes.POINTER(ctypes.c_uint16)
        intp = ctypes.POINTER(ctypes.c_int)
        floatp = ctypes.POINTER(ctypes.c_float)
        sig = {
            "sim_maze_size": ([], ctypes.c_int),
            "sim_goals": ([u8p, u8p, ctypes.c_int], ctypes.c_int),
            "sim_start": ([u8p, u8p], None),
            "sim_new_random": ([ctypes.c_uint32], None),
            "sim_load_file": ([ctypes.c_char_p], ctypes.c_int),
            "sim_reset": ([ctypes.c_int, u16p, u16p], ctypes.c_int),
            "sim_step": ([u8p, u8p], ctypes.c_int),
            "sim_pose": ([u8p, u8p, u8p], None),
            "sim_phase": ([], ctypes.c_int),
            "sim_moves": ([intp, intp], None),
            "sim_wall": ([ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int], ctypes.c_int),
            "sim_cell_known": ([ctypes.c_int, ctypes.c_int], ctypes.c_int),
            "sim_cell_value": ([ctypes.c_int, ctypes.c_int], ctypes.c_uint16),
            "sim_route": ([u8p, u8p, ctypes.c_int, u16p, u16p], ctypes.c_int),
            "sim_run_profile": ([floatp], None),
            "sim_set_search": ([ctypes.c_int, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_int], None),
            "sim_search_params": ([floatp], None),
            "sim_search_time": ([floatp], None),
            "sim_band_count": ([], ctypes.c_int),
            "sim_set_band": ([ctypes.c_int], ctypes.c_int),
            "sim_search_estimate": ([], ctypes.c_float),
            "sim_plant_log": ([ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p,
                               ctypes.c_char_p, ctypes.c_float, ctypes.c_char_p, ctypes.c_int], ctypes.c_int),
            "sim_run_plan": ([ctypes.c_int, u8p, u8p, floatp, ctypes.c_int, floatp, intp], ctypes.c_int),
        }
        for name, (args, res) in sig.items():
            fn = getattr(self.dll, name)
            fn.argtypes = args
            fn.restype = res

        self.size = self.dll.sim_maze_size()
        self._read_goals()
        sx, sy = ctypes.c_uint8(), ctypes.c_uint8()
        self.dll.sim_start(ctypes.byref(sx), ctypes.byref(sy))
        self.start = (sx.value, sy.value)

    def _read_goals(self):
        """今の迷路のゴール(迷路ファイルの G、なければ params.h の MAZE_GOALS)"""
        xs = (ctypes.c_uint8 * 16)()
        ys = (ctypes.c_uint8 * 16)()
        n = self.dll.sim_goals(xs, ys, 16)
        self.goals = [(xs[i], ys[i]) for i in range(n)]

    # --- 本当の迷路 ---
    def new_random(self, seed):
        self.dll.sim_new_random(seed & 0xFFFFFFFF)
        self._read_goals()

    def load_file(self, path):
        """読めたら True。MAZE_SIZE×MAZE_SIZE でない迷路(32×32 など)は False"""
        ok = self.dll.sim_load_file(path.encode("utf-8")) == 1
        if ok:
            self._read_goals()
        return ok

    # --- 探索 ---
    def reset(self, algo, goal_cost=None, back_cost=None):
        """goal_cost/back_cost は (直進, 90°, 180°, 既知区画) か None(既定値)。"""
        def arr(c):
            return None if c is None else (ctypes.c_uint16 * 4)(*c)
        return self.dll.sim_reset(algo, arr(goal_cost), arr(back_cost)) == 1

    def step(self):
        """(状態, 指令の種類, 区画数) を返す。"""
        t, c = ctypes.c_uint8(), ctypes.c_uint8()
        status = self.dll.sim_step(ctypes.byref(t), ctypes.byref(c))
        return status, t.value, c.value

    def pose(self):
        x, y, d = ctypes.c_uint8(), ctypes.c_uint8(), ctypes.c_uint8()
        self.dll.sim_pose(ctypes.byref(x), ctypes.byref(y), ctypes.byref(d))
        return x.value, y.value, d.value

    def phase(self):
        return self.dll.sim_phase()

    def moves(self):
        a, b = ctypes.c_int(), ctypes.c_int()
        self.dll.sim_moves(ctypes.byref(a), ctypes.byref(b))
        return a.value, b.value

    # --- 表示用 ---
    def wall(self, which, x, y, d):
        """which: 0=本当の迷路 1=探索で分かった地図。WALL_OPEN/EXISTS/UNKNOWN を返す。"""
        return self.dll.sim_wall(which, x, y, d)

    def cell_known(self, x, y):
        return self.dll.sim_cell_known(x, y) == 1

    def cell_value(self, x, y):
        """プランナーが最後に計算した値(コスト or 歩数)。行けなければ None。"""
        v = self.dll.sim_cell_value(x, y)
        return None if v == 0xFFFF else v

    def route(self):
        """分かった壁だけでの最短経路。([(種類, 区画数), ...], 見つけたコスト, 真の最短コスト)"""
        types = (ctypes.c_uint8 * 300)()
        cells = (ctypes.c_uint8 * 300)()
        found, best = ctypes.c_uint16(), ctypes.c_uint16()
        n = self.dll.sim_route(types, cells, 300, ctypes.byref(found), ctypes.byref(best))
        return [(types[i], cells[i]) for i in range(n)], found.value, best.value

    # --- 最短走行(時間) ---
    def run_profile(self):
        """最短走行の速さ(set_band で選んだ速度帯)と区画の大きさ[mm]"""
        out = (ctypes.c_float * 8)()
        self.dll.sim_run_profile(out)
        return {"accel": out[0], "vmax": out[1],
                "v_turn": {RUN_SMALL90_R: out[2], RUN_SMALL90_L: out[2],
                           RUN_LARGE90_R: out[3], RUN_LARGE90_L: out[3],
                           RUN_LARGE180_R: out[4], RUN_LARGE180_L: out[4]},
                "section": out[5], "decel": out[6], "band": int(out[7])}

    def band_count(self):
        """最短走行の速度帯(params.h の FAST_BANDS)の数"""
        return self.dll.sim_band_count()

    def set_band(self, band):
        """最短走行の速度帯(1〜)を選ぶ。選んだ番号を返す"""
        return self.dll.sim_set_band(band)

    # --- 探索の設定と時間 ---
    def set_search(self, scope=SCOPE_ROUND, v=0.0, turn_v=0.0, accel=0.0, slalom=True):
        """探索の行き先と速さ(0 なら params.h の値)。次の reset から使う"""
        self.dll.sim_set_search(scope, v, turn_v, accel, 1 if slalom else 0)

    def search_params(self):
        out = (ctypes.c_float * 5)()
        self.dll.sim_search_params(out)
        return {"v": out[0], "turn_v": out[1], "accel": out[2], "slalom": out[3] > 0.5, "scope": int(out[4])}

    def search_estimate(self):
        """今の迷路と探索の設定で、探索を最後までたどった時間の見積もり[s](終わらなければ None)。
        sim_step と同時に(別のスレッドから)呼ばないこと"""
        t = self.dll.sim_search_estimate()
        return t if t > 0 else None

    # --- plant_sim(機体と同じ形式のログ) ---
    def write_maze(self, path):
        """今の本当の迷路を classic 形式のファイルに書く(ゴールの区画に G)。plant_sim に渡すため"""
        n = self.size
        goals = set(self.goals)
        lines = []
        for y in range(n - 1, -1, -1):
            # 北の壁の行
            row = "o"
            for x in range(n):
                row += ("---" if self.wall(0, x, y, 0) == WALL_EXISTS else "   ") + "o"
            lines.append(row)
            # 区画の行(西の壁、区画の真ん中、… 東の外周)
            row = "|" if self.wall(0, 0, y, 3) == WALL_EXISTS else " "
            for x in range(n):
                row += " G " if (x, y) in goals else "   "
                row += "|" if self.wall(0, x, y, 1) == WALL_EXISTS else " "
            lines.append(row)
        row = "o"
        for x in range(n):
            row += ("---" if self.wall(0, x, 0, 2) == WALL_EXISTS else "   ") + "o"
        lines.append(row)
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            f.write("\n".join(lines) + "\n")

    def plant_log(self, maze_path, out_dir, est_s=0.0, build=True, extra=None, exe=None):
        """plant_sim を走らせて、機体と同じ形式のログを out_dir に残す(時間がかかる。別のスレッドから呼んでよい)。
        できたログのパス(.csv か .bin)を返す。失敗なら None"""
        out = ctypes.create_string_buffer(1024)
        self_path = os.path.join(BUILD_DIR, "maze_sim")  # plant_sim.local(tools/maze_sim/)と tools/get_log.py の基準
        enc = (lambda s: None if s is None else os.fsencode(s))
        ok = self.dll.sim_plant_log(enc(maze_path), enc(out_dir), enc(exe), 1 if build else 0, enc(extra),
                                    enc(self_path), est_s or 0.0, out, len(out))
        return os.fsdecode(out.value) if ok and out.value else None

    def search_time(self):
        """探索の時間の見積もり。(ここまでの時間[s], ゴールに止まった/最短経路が決まった時刻[s] か None,
        尻当ての回数, 既知の区間をまとめて走った回数)"""
        out = (ctypes.c_float * 5)()
        self.dll.sim_search_time(out)
        return out[0], (out[1] if out[1] >= 0.0 else None), int(out[2]), int(out[3])

    def run_plan(self, kind):
        """最短走行の指令の列。([(種類, 半区画数), ...], [指令ごとの時間], 合計[s], 加速度が足りるか)。
        行けなければ None。kind は PLAN_TIME / PLAN_COST / PLAN_TIME_BEST。"""
        types = (ctypes.c_uint8 * 300)()
        halves = (ctypes.c_uint8 * 300)()
        times = (ctypes.c_float * 300)()
        total, feasible = ctypes.c_float(), ctypes.c_int()
        n = self.dll.sim_run_plan(kind, types, halves, times, 300, ctypes.byref(total), ctypes.byref(feasible))
        if n == 0:
            return None
        return [(types[i], halves[i]) for i in range(n)], [times[i] for i in range(n)], total.value, feasible.value == 1


if __name__ == "__main__":
    # 動作確認: 乱数迷路で最後まで探索して結果を出す
    sim = MazeSim()
    sim.new_random(1)
    sim.reset(ALGO_DIJKSTRA)
    while True:
        status, _, _ = sim.step()
        if status not in (SIM_MOVED, SIM_AT_GOAL):
            break
    route, found, best = sim.route()
    print("status:", STATUS_NAMES[status], "moves:", sim.moves(), "route cost:", found, "best:", best)
