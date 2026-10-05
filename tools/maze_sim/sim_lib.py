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
PHASE_NAMES = ["to goal", "to start", "done", "failed"]

# ActionType (logic/command.h と同じ)
ACTION_STOP, ACTION_FORWARD, ACTION_TURN_RIGHT, ACTION_TURN_LEFT, ACTION_TURN_BACK = range(5)
ACTION_NAMES = ["STOP", "FORWARD", "RIGHT", "LEFT", "BACK"]
ACTION_QUARTER_TURNS = [0, 0, 1, -1, 2]

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
    srcs = [os.path.join(HERE, "sim_api.c"), os.path.join(HERE, "sim_core.c"),
            os.path.join(ROOT, "Core", "Src", "logic", "command.c")]
    srcs += sorted(glob.glob(os.path.join(ROOT, "Core", "Src", "logic", "maze", "*.c")))
    return srcs


def _hashed_files():
    """ビルド結果に影響するファイル(ソースと、それが読むヘッダ)"""
    files = _sources()
    files += [os.path.join(HERE, "sim_core.h"),
              os.path.join(ROOT, "Core", "Inc", "params.h"),
              os.path.join(ROOT, "Core", "Inc", "global.h"),
              os.path.join(ROOT, "Core", "Inc", "logic", "command.h")]
    files += sorted(glob.glob(os.path.join(ROOT, "Core", "Inc", "logic", "maze", "*.h")))
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
    cmd = ["gcc"] + flags + ["-o", lib] + _sources()
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
        }
        for name, (args, res) in sig.items():
            fn = getattr(self.dll, name)
            fn.argtypes = args
            fn.restype = res

        self.size = self.dll.sim_maze_size()
        xs = (ctypes.c_uint8 * 16)()
        ys = (ctypes.c_uint8 * 16)()
        n = self.dll.sim_goals(xs, ys, 16)
        self.goals = [(xs[i], ys[i]) for i in range(n)]
        sx, sy = ctypes.c_uint8(), ctypes.c_uint8()
        self.dll.sim_start(ctypes.byref(sx), ctypes.byref(sy))
        self.start = (sx.value, sy.value)

    # --- 本当の迷路 ---
    def new_random(self, seed):
        self.dll.sim_new_random(seed & 0xFFFFFFFF)

    def load_file(self, path):
        return self.dll.sim_load_file(path.encode("utf-8")) == 1

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
