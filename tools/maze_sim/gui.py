"""yuho の迷路探索を pygame で見るGUI。

探索の判断はファームウェアと同じ logic 層 (SearchPlanner) が行う (sim_lib.py 経由)。
迷路の分類・お気に入りは maze_catalog.py が行う。このファイルは描画と操作だけを受け持つ。

使い方:
    python tools/maze_sim/gui.py                    前回開いた迷路(なければ乱数の迷路)
    python tools/maze_sim/gui.py --random 7         シード7の乱数迷路
    python tools/maze_sim/gui.py maze.txt           迷路ファイル(classic 形式)
オプション:
    --algo dijkstra|adachi     探索のアルゴリズム
    --goal S,T90,T180,K        行きのコスト(直進,90°,180°,既知区画の上乗せ)
    --back S,T90,T180,K        帰りのコスト
    --speed N                  1秒あたりの区画数(既定 8)
    --scope round|oneway|full  探索の行き先(往復・片道・全面)
    --search V,TV,ACCEL        探索の直進の速さ・スラロームの速さ・加速度(既定 params.h)
    --pivot                    探索で曲がるとき超信地旋回
    --band N                   最短走行の速度帯(FAST_BANDS の N 番目。既定 3)
    --plant-no-build           X で plant_sim を走らせる前にビルドし直さない
    --plant-opt "..."          X で plant_sim にそのまま渡すオプション(例 "--no-crash-stop")

操作(画面右にも出る):
    SPACE / P   再生・一時停止          S / →   1区画だけ進める(一時停止中)
    F           最後まで一気に進める    R       同じ迷路で最初から
    L           迷路の一覧(大会・年・予選/決勝で分類、お気に入り・最近開いた迷路)
    M           今の迷路をお気に入りに登録 / 解除
    N / B       次 / 前の迷路(一覧の同じ分類の中で。乱数ならシード)
    A           アルゴリズムを切り替えて最初から
    C           コスト(歩数)の表示      T       まだ見ていない壁の表示
    O           探索の行き先(往復 → 片道 → 全面)を切り替えて最初から
    K           最短走行の速度帯を次の帯にする
    X           今の迷路と探索の設定で plant_sim を走らせ、機体と同じ形式のログを残す(build/plant_logs/)
    Z           最後にできた plant_sim のログを log_viewer で開く
    ↑ / ↓       速さ                   ESC / Q 終わる
探索が終わった後(最短走行):
    G           最短走行を再生 / 一時停止(終わっていれば最初から)
    V           表示する経路を切り替え(時間で最短 / コストで最短+大回り / 全部知っていたら)
    SPACE       再生中は一時停止        ↑ / ↓   再生中は再生の速さ(実時間の何倍か)
迷路の一覧の中:
    ↑ / ↓ / PageUp / PageDown  選ぶ(押し続けると連続で動く。選んだ迷路は左にプレビューが出る)
    → / Enter  分類に入る・迷路を開く    ← / BackSpace  分類の一覧へ戻る
    M          お気に入りに登録 / 解除    ESC / L        一覧を閉じる
"""

import argparse
import math
import os
import subprocess
import sys
import threading
import time

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")  # pygame の起動メッセージを出さない
import pygame  # noqa: E402

import maze_catalog as mc  # noqa: E402
import run_motion as rm  # noqa: E402
import sim_lib as sl  # noqa: E402

CELL = 36       # 1区画の大きさ[px]
MARGIN = 28
PANEL_W = 400   # 右側の情報欄の幅
FPS = 60

# 迷路の一覧で、移動キーを押し続けたときの連続移動
REPEAT_KEYS = (pygame.K_UP, pygame.K_DOWN, pygame.K_PAGEUP, pygame.K_PAGEDOWN)
REPEAT_DELAY_S = 0.35     # 押してから連続移動が始まるまで
REPEAT_INTERVAL_S = 0.05  # 連続移動の間隔(1秒に20行)

COLOR_BG = (24, 24, 28)
COLOR_GRID = (52, 52, 58)
COLOR_KNOWN_CELL = (40, 64, 44)      # 4方向とも壁が分かった区画
COLOR_GOAL = (90, 70, 20)
COLOR_START = (30, 50, 90)
COLOR_WALL_KNOWN = (235, 60, 60)     # 見つけた壁
COLOR_WALL_HIDDEN = (90, 90, 96)     # まだ見ていない本当の壁
COLOR_WALL_PREVIEW = (210, 210, 220) # 一覧のプレビューの壁
COLOR_MOUSE = (0, 200, 255)
COLOR_MOUSE_DIR = (255, 230, 80)
COLOR_TRAIL = (0, 120, 160)
COLOR_ROUTE = (255, 210, 60)        # 最短走行の直進
COLOR_RUN_SMALL = (255, 130, 50)     # 最短走行の小回り
COLOR_RUN_LARGE = (210, 110, 255)    # 最短走行の大回り
COLOR_TEXT = (225, 225, 230)
COLOR_DIM = (140, 140, 150)
COLOR_BAD = (255, 90, 90)
COLOR_GOOD = (120, 230, 120)
COLOR_SELECT = (60, 70, 110)
COLOR_STAR = (255, 210, 60)

JP_FONTS = ["bizudgothic", "meiryo", "yugothic", "msgothic"]


def parse_cost(text):
    parts = text.split(",")
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("cost must be S,T90,T180,K")
    return tuple(int(p) for p in parts)


def parse_search(text):
    try:
        v = tuple(float(x) for x in text.split(","))
    except ValueError:
        v = ()
    if len(v) != 3 or min(v) <= 0:
        raise argparse.ArgumentTypeError("search must be V,TURN_V,ACCEL (all > 0)")
    return v


def jp_font(size):
    """日本語が出るフォント(なければ既定のフォント)"""
    for name in JP_FONTS:
        if pygame.font.match_font(name):
            return pygame.font.SysFont(name, size)
    return pygame.font.SysFont(None, size + 4)


class MazeGui:
    def __init__(self, args):
        self.sim = sl.MazeSim()
        self.n = self.sim.size
        self.algo = sl.ALGO_ADACHI if args.algo == "adachi" else sl.ALGO_DIJKSTRA
        self.goal_cost = args.goal
        self.back_cost = args.back
        self.speed = args.speed
        # 探索の行き先と速さ(機体の RUN の SEARCH と同じ選び方)、最短走行の速度帯(FAST_BANDS)
        self.scope = {"round": sl.SCOPE_ROUND, "oneway": sl.SCOPE_ONE_WAY, "full": sl.SCOPE_FULL}[args.scope]
        self.search_speeds = args.search or (0.0, 0.0, 0.0)  # 0 は params.h の値
        self.slalom = not args.pivot
        self.band = args.band
        self.plan_kind = sl.PLAN_TIME  # 表示・再生する最短走行の経路
        self.run_scale = 1.0           # 再生の速さ(実時間の何倍か)

        # 迷路の一覧(my_mazes/ と mazes/)とお気に入り
        self.entries = mc.load_entries()
        self.prefs = mc.load_prefs()

        # 開く迷路: 引数 → (どちらもなければ)前回開いた迷路 → 乱数の迷路
        self.maze_file = args.maze_file
        self.seed = args.random if args.random is not None else 1
        if self.maze_file is None and args.random is None:
            by_id = {e.id: e for e in self.entries}
            recent = [by_id[i] for i in self.prefs["recent"] if i in by_id and by_id[i].playable]
            if recent:
                self.maze_file = recent[0].path

        # plant_sim(機体と同じ形式のログ。X で走らせ、Z で log_viewer で開く)
        self.plant_build = not args.plant_no_build
        self.plant_opt = args.plant_opt
        self.plant_thread = None   # 走らせているスレッド
        self.plant_result = None   # スレッドが書く: できたログのパス(失敗なら "")
        self.plant_status = ""     # 右の欄に出す文
        self.plant_log = None      # 最後にできたログ
        self.plant_started = 0.0

        self.show_cost = True
        self.show_hidden = True
        self.paused = True
        self.message = ""

        # 迷路の一覧の状態
        self.browser_open = False
        self.browser_path = []     # 今いる場所(一覧の木のノードのキーを根から並べたもの)。[] なら一番上
        self.browser_index = 0
        self.browser_scroll = 0
        self.browser_msg = ""
        self.preview_cache = {}
        self.repeat_key = None     # 押し続けている移動キー(迷路の一覧の中だけ)
        self.repeat_timer = 0.0    # 次に連続移動するまでの残り時間[s]

        pygame.init()
        maze_px = self.n * CELL
        self.screen = pygame.display.set_mode((MARGIN * 2 + maze_px + PANEL_W, MARGIN * 2 + maze_px))
        pygame.display.set_caption("yuho maze simulator")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("consolas", 15)
        self.font_small = pygame.font.SysFont("consolas", 11)
        self.font_big = pygame.font.SysFont("consolas", 20, bold=True)
        self.font_jp = jp_font(15)
        self.font_jp_small = jp_font(12)

        self.apply_search()
        self.band = self.sim.set_band(self.band)
        self.prof = self.sim.run_profile()
        self.load_maze()

    # ------------------------------------------------------------
    # 迷路と探索の状態
    # ------------------------------------------------------------

    def current_entry(self):
        """今の迷路が一覧(my_mazes/ か mazes/)のものなら、その MazeEntry"""
        if not self.maze_file:
            return None
        target = os.path.normcase(os.path.abspath(self.maze_file))
        for e in self.entries:
            if os.path.normcase(os.path.abspath(e.path)) == target:
                return e
        return None

    def load_maze(self):
        if self.maze_file:
            if not self.sim.load_file(self.maze_file):
                print(f"cannot load {self.maze_file} (not a {self.n}x{self.n} maze?)", file=sys.stderr)
                sys.exit(2)
            e = self.current_entry()
            if e:
                mc.add_recent(self.prefs, e)
        else:
            self.sim.new_random(self.seed)
        self.restart()

    def restart(self):
        if not self.sim.reset(self.algo, self.goal_cost, self.back_cost):
            print("invalid cost (straight must be >= 1, total must fit in 13 bits)", file=sys.stderr)
            sys.exit(2)
        self.status = sl.SIM_MOVED
        self.last_action = None
        self.pose = self.sim.pose()
        self.prev_pose = self.pose
        self.anim = 1.0           # 0→1 で prev_pose から pose へ動く
        self.trail = [self.pose[:2]]
        self.step_timer = 0.0
        self.route = None         # 探索が終わったら (指令の列, 見つけたコスト, 真の最短)
        self.plans = None         # 探索が終わったら {kind: (合計の時間, 加速度が足りるか, RunMotion)}
        self.run_t = None         # 最短走行の再生の時刻[s](None なら再生していない)
        self.run_playing = False
        self.message = ""

    def apply_search(self):
        """探索の行き先と速さを DLL に渡す(次の reset から使う)"""
        v, turn_v, accel = self.search_speeds
        self.sim.set_search(self.scope, v, turn_v, accel, self.slalom)

    def compute_plans(self):
        """探索の後の最短走行の経路と時間(今の速度帯で)"""
        self.plans = {}
        for kind in (sl.PLAN_TIME, sl.PLAN_COST, sl.PLAN_TIME_BEST):
            plan = self.sim.run_plan(kind)
            if plan:
                cmds, times, total, feasible = plan
                motion = rm.RunMotion(cmds, times, self.prof, self.sim.start)
                self.plans[kind] = (total, feasible, motion)

    def start_plant(self):
        """今の迷路と探索の設定で plant_sim を走らせる(別のスレッド。走っている間も GUI は動く)"""
        if self.plant_thread is not None:
            self.message = "plant_sim is already running"
            return
        e = self.current_entry()
        name = e.name if e else (os.path.splitext(os.path.basename(self.maze_file))[0] if self.maze_file
                                 else f"random{self.seed}")
        scope = sl.SCOPE_NAMES[self.scope]
        out_dir = os.path.join(sl.BUILD_DIR, "plant_logs", f"{name}_{scope}_{time.strftime('%Y%m%d_%H%M%S')}")
        maze_path = os.path.join(out_dir, "maze.txt")
        self.sim.write_maze(maze_path)        # 乱数の迷路でも渡せるように、今の迷路を書き出す
        est = self.sim.search_estimate()      # logic 層の計算なので、このスレッドで先に行う
        self.plant_result = None
        self.plant_started = time.time()
        self.plant_status = "running" + (f" (search est. {est:.0f} s)" if est else "")

        def work():
            log = self.sim.plant_log(maze_path, out_dir, est or 0.0, self.plant_build, self.plant_opt)
            self.plant_result = log or ""

        self.plant_thread = threading.Thread(target=work, daemon=True)
        self.plant_thread.start()

    def poll_plant(self):
        """plant_sim のスレッドが終わっていたら結果を受け取る"""
        if self.plant_thread is None or self.plant_thread.is_alive():
            return
        self.plant_thread = None
        if self.plant_result:
            self.plant_log = self.plant_result
            self.plant_status = f"done: {os.path.basename(self.plant_log)} (Z: view)"
            self.message = "plant_sim log: " + os.path.relpath(self.plant_log, sl.ROOT)
        else:
            self.plant_status = "FAILED (see the console)"

    def open_plant_log(self):
        """最後にできた plant_sim のログを log_viewer で開く(別のプロセス)"""
        if not self.plant_log:
            self.message = "no plant_sim log yet (X: run plant_sim)"
            return
        viewer = os.path.join(sl.ROOT, "tools", "log_viewer.py")
        subprocess.Popen([sys.executable, viewer, self.plant_log])

    def finished(self):
        return self.status not in (sl.SIM_MOVED, sl.SIM_AT_GOAL)

    def do_step(self):
        if self.finished():
            return
        self.prev_pose = self.pose
        self.status, action, cells = self.sim.step()
        self.pose = self.sim.pose()
        self.last_action = (action, cells)
        self.anim = 0.0
        if self.pose[:2] != self.trail[-1]:
            self.trail.append(self.pose[:2])

        if self.status == sl.SIM_AT_GOAL:
            self.message = "reached goal"
        elif self.status == sl.SIM_DONE:
            self.route = self.sim.route()
            self.compute_plans()
            self.message = "search done (G: fastest run)"
            self.paused = True
        elif self.finished():
            self.message = sl.STATUS_NAMES[self.status]
            self.paused = True

    def current_motion(self):
        if not self.plans or self.plan_kind not in self.plans:
            return None
        return self.plans[self.plan_kind][2]

    def toggle_run(self):
        """最短走行の再生 / 一時停止。終わっていれば最初から"""
        motion = self.current_motion()
        if motion is None:
            return
        if self.run_t is None or self.run_t >= motion.duration:
            self.run_t = 0.0
            self.run_playing = True
        else:
            self.run_playing = not self.run_playing

    def run_to_end(self):
        while not self.finished():
            self.do_step()
        self.anim = 1.0

    # ------------------------------------------------------------
    # 迷路の一覧(分類・お気に入り)
    # ------------------------------------------------------------

    def browser_tree(self):
        return mc.build_tree(self.entries, self.prefs)

    def browser_rows(self):
        """今の階層に並ぶもの(ノードの dict か MazeEntry)"""
        tree = self.browser_tree()
        if not self.browser_path:
            return tree
        node = mc.find_node(tree, self.browser_path[-1])
        if node is None:
            self.browser_path = []
            return tree
        return node["children"] if "children" in node else node["items"]

    @staticmethod
    def first_entry(row):
        """ノードならその中の最初の迷路、迷路ならそのもの"""
        while isinstance(row, dict):
            inner = row.get("items") or row.get("children") or []
            if not inner:
                return None
            row = inner[0]
        return row

    def open_browser(self):
        if not self.entries:
            self.message = "no mazes: run maze_batch.py or fetch_mazes.sh"
            return
        self.browser_open = True
        self.browser_scroll = 0
        self.browser_msg = ""
        # 今の迷路があれば、その分類の中でその迷路を選んだ状態で開く
        e = self.current_entry()
        if e:
            self.browser_path = [e.node_key] if e.mine else [mc.IMPORTED_KEY, e.node_key]
            ids = [r.id for r in self.browser_rows() if not isinstance(r, dict)]
            self.browser_index = ids.index(e.id) if e.id in ids else 0
        else:
            self.browser_path = []
            self.browser_index = 0

    def browser_selected_entry(self):
        """プレビューする迷路(ノードを選んでいるときはその中の最初の迷路)"""
        rows = self.browser_rows()
        if not rows:
            return None
        return self.first_entry(rows[min(self.browser_index, len(rows) - 1)])

    def browser_key(self, key):
        rows = self.browser_rows()
        count = len(rows)
        self.browser_msg = ""
        if key in (pygame.K_ESCAPE, pygame.K_l):
            self.browser_open = False
        elif key == pygame.K_UP:
            self.browser_index = max(0, self.browser_index - 1)
        elif key == pygame.K_DOWN:
            self.browser_index = min(count - 1, self.browser_index + 1)
        elif key == pygame.K_PAGEUP:
            self.browser_index = max(0, self.browser_index - 10)
        elif key == pygame.K_PAGEDOWN:
            self.browser_index = min(count - 1, self.browser_index + 10)
        elif key in (pygame.K_RIGHT, pygame.K_RETURN, pygame.K_KP_ENTER) and count > 0:
            row = rows[self.browser_index]
            if isinstance(row, dict):
                self.browser_path.append(row["key"])
                self.browser_index = 0
                self.browser_scroll = 0
            elif not row.playable:
                self.browser_msg = f"{row.size}×{row.size} はシミュレータ未対応(プレビューのみ)"
            else:
                self.maze_file = row.path
                self.browser_open = False
                self.load_maze()
        elif key in (pygame.K_LEFT, pygame.K_BACKSPACE) and self.browser_path:
            left = self.browser_path.pop()
            keys = [r["key"] for r in self.browser_rows() if isinstance(r, dict)]
            self.browser_index = keys.index(left) if left in keys else 0
            self.browser_scroll = 0
        elif key == pygame.K_m and count > 0 and not isinstance(rows[self.browser_index], dict):
            mc.toggle_favorite(self.prefs, rows[self.browser_index])
            # お気に入りの中で解除したら、その行は消えるので位置を詰める
            self.browser_index = min(self.browser_index, max(0, len(self.browser_rows()) - 1))

    def neighbor_file(self, step):
        """一覧の同じ分類の中で step 個先の(開ける)迷路。一覧にないファイルなら同じフォルダの名前順"""
        e = self.current_entry()
        if e:
            node = mc.find_node(self.browser_tree(), e.node_key)
            items = [x for x in node["items"] if x.playable] if node else []
            ids = [x.id for x in items]
            if e.id in ids:
                return items[(ids.index(e.id) + step) % len(items)].path
        folder = os.path.dirname(os.path.abspath(self.maze_file))
        names = sorted(f for f in os.listdir(folder) if f.lower().endswith(".txt"))
        current = os.path.basename(self.maze_file)
        i = names.index(current) if current in names else 0
        return os.path.join(folder, names[(i + step) % len(names)])

    # ------------------------------------------------------------
    # 座標
    # ------------------------------------------------------------

    def cell_rect(self, x, y):
        px = MARGIN + x * CELL
        py = MARGIN + (self.n - 1 - y) * CELL
        return pygame.Rect(px, py, CELL, CELL)

    def cell_center(self, x, y):
        r = self.cell_rect(x, y)
        return r.centerx, r.centery

    def mm_to_px(self, p):
        """最短走行の座標[mm](スタート区画の左下が原点、北が +y)を画面の座標へ"""
        s = CELL / self.prof["section"]
        return MARGIN + p[0] * s, MARGIN + self.n * CELL - p[1] * s

    def wall_line(self, x, y, d):
        r = self.cell_rect(x, y)
        if d == 0:
            return (r.left, r.top), (r.right, r.top)
        if d == 1:
            return (r.right, r.top), (r.right, r.bottom)
        if d == 2:
            return (r.left, r.bottom), (r.right, r.bottom)
        return (r.left, r.top), (r.left, r.bottom)

    # ------------------------------------------------------------
    # 描画
    # ------------------------------------------------------------

    def draw_grid(self):
        for i in range(self.n + 1):
            pygame.draw.line(self.screen, COLOR_GRID, (MARGIN + i * CELL, MARGIN),
                             (MARGIN + i * CELL, MARGIN + self.n * CELL))
            pygame.draw.line(self.screen, COLOR_GRID, (MARGIN, MARGIN + i * CELL),
                             (MARGIN + self.n * CELL, MARGIN + i * CELL))

    def draw_start_goal(self):
        for (x, y) in self.sim.goals:
            pygame.draw.rect(self.screen, COLOR_GOAL, self.cell_rect(x, y).inflate(-2, -2))
        sx, sy = self.sim.start
        pygame.draw.rect(self.screen, COLOR_START, self.cell_rect(sx, sy).inflate(-2, -2))

    def draw_cells(self):
        for y in range(self.n):
            for x in range(self.n):
                if (x, y) in self.sim.goals or (x, y) == self.sim.start:
                    continue
                if self.sim.cell_known(x, y):
                    pygame.draw.rect(self.screen, COLOR_KNOWN_CELL, self.cell_rect(x, y).inflate(-2, -2))
        self.draw_start_goal()
        self.draw_grid()

    def draw_cost(self):
        if not self.show_cost:
            return
        for y in range(self.n):
            for x in range(self.n):
                v = self.sim.cell_value(x, y)
                if v is None:
                    continue
                img = self.font_small.render(str(v), True, COLOR_DIM)
                cx, cy = self.cell_center(x, y)
                self.screen.blit(img, img.get_rect(center=(cx, cy)))

    def draw_walls(self):
        # 北と東だけ見れば内壁を1回ずつ、外周の南と西は別に描く
        for y in range(self.n):
            for x in range(self.n):
                dirs = [0, 1]
                if y == 0:
                    dirs.append(2)
                if x == 0:
                    dirs.append(3)
                for d in dirs:
                    truth = self.sim.wall(0, x, y, d)
                    known = self.sim.wall(1, x, y, d)
                    if known == sl.WALL_EXISTS:
                        a, b = self.wall_line(x, y, d)
                        pygame.draw.line(self.screen, COLOR_WALL_KNOWN, a, b, 4)
                    elif known == sl.WALL_UNKNOWN and truth == sl.WALL_EXISTS and self.show_hidden:
                        a, b = self.wall_line(x, y, d)
                        pygame.draw.line(self.screen, COLOR_WALL_HIDDEN, a, b, 3)

    def draw_trail(self):
        if len(self.trail) < 2:
            return
        pts = [self.cell_center(x, y) for x, y in self.trail]
        pygame.draw.lines(self.screen, COLOR_TRAIL, False, pts, 2)

    def draw_route(self):
        """最短走行の経路(V で切り替え)。固定の形で、直進・小回り・大回りで色を分ける"""
        motion = self.current_motion()
        if motion is None:
            return
        for seg in motion.segments:
            if seg.type == sl.RUN_STRAIGHT:
                color = COLOR_ROUTE
            elif seg.type in (sl.RUN_SMALL90_R, sl.RUN_SMALL90_L):
                color = COLOR_RUN_SMALL
            else:
                color = COLOR_RUN_LARGE
            pygame.draw.lines(self.screen, color, False, [self.mm_to_px(p) for p in seg.points()], 3)

    def draw_mouse(self):
        motion = self.current_motion()
        if self.run_t is not None and motion is not None:
            pos, heading, _, _ = motion.state_at(self.run_t)
            cx, cy = self.mm_to_px(pos)
            r = CELL * 0.28
            pygame.draw.circle(self.screen, COLOR_MOUSE, (int(cx), int(cy)), int(r))
            tip = (cx + math.cos(heading) * r, cy - math.sin(heading) * r)
            pygame.draw.line(self.screen, COLOR_MOUSE_DIR, (cx, cy), tip, 3)
            return
        x0, y0, _ = self.prev_pose
        x1, y1, d = self.pose
        a = min(self.anim, 1.0)
        c0 = self.cell_center(x0, y0)
        c1 = self.cell_center(x1, y1)
        cx = c0[0] + (c1[0] - c0[0]) * a
        cy = c0[1] + (c1[1] - c0[1]) * a
        r = CELL * 0.28
        pygame.draw.circle(self.screen, COLOR_MOUSE, (int(cx), int(cy)), int(r))
        tip = (cx + sl.DIR_DX[d] * r, cy - sl.DIR_DY[d] * r)
        pygame.draw.line(self.screen, COLOR_MOUSE_DIR, (cx, cy), tip, 3)

    def draw_preview(self, entry):
        """一覧で選んでいる迷路を、全部の壁が見えた状態で描く(32×32 なども縮めて描く)"""
        if entry is None:
            self.draw_start_goal()
            self.draw_grid()
            return
        if entry.path not in self.preview_cache:
            self.preview_cache[entry.path] = mc.read_maze(entry.path)
        maze = self.preview_cache[entry.path]
        if maze is None:
            return
        north, east, goals = maze
        n = len(north)
        cs = CELL * self.n / n  # 画面上の区画の大きさ

        def rect(x, y):
            return pygame.Rect(int(MARGIN + x * cs), int(MARGIN + (n - 1 - y) * cs), int(cs) + 1, int(cs) + 1)

        for (x, y) in goals:
            pygame.draw.rect(self.screen, COLOR_GOAL, rect(x, y).inflate(-2, -2))
        pygame.draw.rect(self.screen, COLOR_START, rect(0, 0).inflate(-2, -2))
        size_px = self.n * CELL
        for i in range(n + 1):
            v = int(MARGIN + i * cs)
            pygame.draw.line(self.screen, COLOR_GRID, (v, MARGIN), (v, MARGIN + size_px))
            pygame.draw.line(self.screen, COLOR_GRID, (MARGIN, v), (MARGIN + size_px, v))

        width = 3 if n <= 16 else 2
        for y in range(n):
            for x in range(n):
                r = rect(x, y)
                if north[y][x] or y == n - 1:
                    pygame.draw.line(self.screen, COLOR_WALL_PREVIEW, r.topleft, (r.left + int(cs), r.top), width)
                if east[y][x] or x == n - 1:
                    pygame.draw.line(self.screen, COLOR_WALL_PREVIEW, (r.left + int(cs), r.top),
                                     (r.left + int(cs), r.top + int(cs)), width)
                if y == 0:
                    pygame.draw.line(self.screen, COLOR_WALL_PREVIEW, (r.left, r.top + int(cs)),
                                     (r.left + int(cs), r.top + int(cs)), width)
                if x == 0:
                    pygame.draw.line(self.screen, COLOR_WALL_PREVIEW, r.topleft, (r.left, r.top + int(cs)), width)

    def blit(self, text, font, color, pos):
        self.screen.blit(font.render(text, True, color), pos)

    def draw_browser(self):
        x0 = MARGIN * 2 + self.n * CELL
        width = PANEL_W - MARGIN
        y = MARGIN
        tree = self.browser_tree()
        title = "迷路の一覧"
        for key in self.browser_path:
            node = mc.find_node(tree, key)
            if node:
                title += " > " + node["label"]
        self.blit(title, self.font_jp, COLOR_TEXT, (x0, y))
        y += self.font_jp.get_linesize() + 6

        help_lines = ["↑↓ 選ぶ  →/Enter 開く  ← 戻る", "M お気に入り  ESC/L 閉じる"]
        if self.browser_msg:
            help_lines.insert(0, self.browser_msg)
        bottom = self.screen.get_height() - MARGIN - len(help_lines) * self.font_jp_small.get_linesize()

        rows = self.browser_rows()
        has_entries = any(not isinstance(r, dict) for r in rows)
        row_h = self.font_jp.get_linesize() + (self.font_jp_small.get_linesize() if has_entries else 0) + 4
        visible = max(1, (bottom - y) // row_h)
        # 選んでいる行が見えるようにずらす
        if self.browser_index < self.browser_scroll:
            self.browser_scroll = self.browser_index
        elif self.browser_index >= self.browser_scroll + visible:
            self.browser_scroll = self.browser_index - visible + 1

        for i in range(self.browser_scroll, min(len(rows), self.browser_scroll + visible)):
            rect = pygame.Rect(x0 - 4, y, width, row_h - 2)
            if i == self.browser_index:
                pygame.draw.rect(self.screen, COLOR_SELECT, rect)
            row = rows[i]
            if isinstance(row, dict):
                mark = " ▶" if "children" in row else ""
                self.blit(f"{row['label']}  ({mc.node_count(row)}){mark}", self.font_jp, COLOR_TEXT, (x0, y + 2))
            else:
                star = "★ " if mc.is_favorite(self.prefs, row) else "   "
                color = COLOR_TEXT if row.playable else COLOR_DIM
                self.blit(star, self.font_jp, COLOR_STAR, (x0, y + 2))
                self.blit(row.label, self.font_jp, color, (x0 + 22, y + 2))
                self.blit(row.name, self.font_jp_small, COLOR_DIM, (x0 + 22, y + 2 + self.font_jp.get_linesize()))
            y += row_h

        if len(rows) > visible:
            self.blit(f"{self.browser_index + 1} / {len(rows)}", self.font_small, COLOR_DIM, (x0 + width - 60, MARGIN))

        y = bottom
        for line in help_lines:
            color = COLOR_BAD if line == self.browser_msg else COLOR_DIM
            self.blit(line, self.font_jp_small, color, (x0, y))
            y += self.font_jp_small.get_linesize()

    def draw_panel(self):
        x0 = MARGIN * 2 + self.n * CELL
        y = MARGIN
        lines = []

        def add(text, color=COLOR_TEXT, font=None):
            lines.append((text, color, font or self.font))

        add("yuho maze simulator", COLOR_TEXT, self.font_big)
        add("")
        e = self.current_entry()
        if e:
            star = "★ " if mc.is_favorite(self.prefs, e) else ""
            where = "" if e.mine else "インポート: "
            add(f"{star}{where}{e.label}", COLOR_STAR if star else COLOR_TEXT, self.font_jp)
            add(f"maze   : {e.name}")
        elif self.maze_file:
            add(f"maze   : {os.path.basename(self.maze_file)}")
        else:
            add(f"maze   : random seed {self.seed}")
        add(f"algo   : {sl.ALGO_NAMES[self.algo]}")
        if self.algo == sl.ALGO_DIJKSTRA:
            add(f"goal   : {self.cost_text(self.goal_cost, False)}")
            add(f"back   : {self.cost_text(self.back_cost, True)}")
        add(f"speed  : {self.speed} cells/s")
        sp = self.sim.search_params()
        add(f"search : {sl.SCOPE_NAMES[sp['scope']]}, v {sp['v']:.0f} / slalom {sp['turn_v']:.0f}"
            + ("" if sp['slalom'] else " (pivot)"))
        # 区画の数字は、プランナーが最後に計算した目的地までの値
        target = {sl.PHASE_TO_GOAL: "goal", sl.PHASE_FULL: "targets"}.get(self.sim.phase(), "start")
        unit = "steps" if self.algo == sl.ALGO_ADACHI else "cost"
        add(f"numbers: {unit} to {target}", COLOR_DIM)
        add("")

        phase = sl.PHASE_NAMES[self.sim.phase()]
        bad = self.status in (sl.SIM_FAILED, sl.SIM_CRASH, sl.SIM_LOST)
        add(f"phase  : {phase}", COLOR_BAD if bad else COLOR_TEXT)
        x, y_, d = self.pose
        add(f"pos    : ({x},{y_}) {'NESW'[d]}")
        if self.last_action:
            t, cells = self.last_action
            name = sl.ACTION_NAMES[t] + (f" x{cells}" if t == sl.ACTION_FORWARD else "")
            add(f"action : {name}")
        to_goal, back = self.sim.moves()
        add(f"moves  : {to_goal} + {back} = {to_goal + back}")
        # 探索の時間の見積もり(機体の動きを params.h の値でたどる。search_time.c)
        t, t_goal, n_setpos, n_known = self.sim.search_time()
        goal_label = "route decided" if sp['scope'] == sl.SCOPE_FULL else "goal"
        add(f"time   : {t:5.1f} s" + (f" ({goal_label} {t_goal:.1f} s)" if t_goal is not None else ""))
        add(f"         setpos {n_setpos}, known runs {n_known}", COLOR_DIM)
        if self.plant_status:
            status = self.plant_status
            if self.plant_thread is not None:
                status += f" {time.time() - self.plant_started:.0f} s"
            add(f"plant  : {status}", COLOR_BAD if status.startswith("FAILED") else COLOR_TEXT)
        if self.message:
            add(f">> {self.message}", COLOR_BAD if bad else COLOR_GOOD)

        if self.route:
            _, found, best = self.route
            add("")
            add(f"route cost {found}" + (" = optimal" if found == best else f" (best {best})"),
                COLOR_GOOD if found == best else COLOR_TEXT)
        if self.plans:
            add(f"fastest run band {self.band}: v {self.prof['vmax']:.0f}, "
                f"small {self.prof['v_turn'][sl.RUN_SMALL90_R]:.0f}")
            add("  (V: switch, G: play, K: band)", COLOR_DIM)
            best_time = self.plans[sl.PLAN_TIME_BEST][0] if sl.PLAN_TIME_BEST in self.plans else None
            for kind, label in ((sl.PLAN_TIME, "time-optimal"), (sl.PLAN_COST, "cost + large"),
                                (sl.PLAN_TIME_BEST, "all walls known")):
                mark = ">" if kind == self.plan_kind else " "
                if kind not in self.plans:
                    add(f"{mark} {label:16s} no route", COLOR_BAD)
                    continue
                total, feasible, _ = self.plans[kind]
                color = COLOR_TEXT if kind == self.plan_kind else COLOR_DIM
                if not feasible:
                    color = COLOR_BAD
                elif kind == sl.PLAN_TIME and best_time is not None and total <= best_time + 1e-4:
                    color = COLOR_GOOD
                add(f"{mark} {label:16s} {total:7.3f} s" + ("" if feasible else " (accel!)"), color)
            motion = self.current_motion()
            if self.run_t is not None and motion is not None:
                _, _, v, i = motion.state_at(self.run_t)
                add(f"  t {self.run_t:6.3f} s  x{self.run_scale:g}  v {v:5.0f} mm/s")
                add(f"  {sl.RUN_NAMES[motion.segments[i].type]}", COLOR_DIM)

        add("")
        if self.run_t is not None:
            add("RUN PLAYING" if self.run_playing else "RUN PAUSED", COLOR_GOOD if self.run_playing else COLOR_DIM)
        else:
            add("PAUSED" if self.paused else "RUNNING", COLOR_GOOD if not self.paused else COLOR_DIM)
        for k in ["SPACE/P play  S/-> step  F to end",
                  "R restart  L list  M fav  N/B maze",
                  f"A algo  C cost[{'on' if self.show_cost else 'off'}]  T hidden[{'on' if self.show_hidden else 'off'}]",
                  "O scope  K band  G run  V plan",
                  "X plant_sim log  Z view the log",
                  "UP/DOWN speed  Q quit"]:
            add(k, COLOR_DIM)

        for text, color, font in lines:
            if text:
                self.screen.blit(font.render(text, True, color), (x0, y))
            y += font.get_linesize()

    def cost_text(self, cost, back):
        if cost is None:
            return "params.h" + (" (+known)" if back else "")
        return ",".join(str(c) for c in cost)

    def draw(self):
        self.screen.fill(COLOR_BG)
        if self.browser_open:
            self.draw_preview(self.browser_selected_entry())
            self.draw_browser()
        else:
            self.draw_cells()
            self.draw_cost()
            self.draw_trail()
            self.draw_route()
            self.draw_walls()
            self.draw_mouse()
            self.draw_panel()
        pygame.display.flip()

    # ------------------------------------------------------------
    # 操作
    # ------------------------------------------------------------

    def handle_key(self, key):
        if self.browser_open:
            self.browser_key(key)
            return True
        if key in (pygame.K_ESCAPE, pygame.K_q):
            return False
        if key in (pygame.K_SPACE, pygame.K_p):
            if self.run_t is not None:
                self.toggle_run()
            elif not self.finished():
                self.paused = not self.paused
        elif key == pygame.K_g:
            self.toggle_run()
        elif key == pygame.K_v:
            if self.plans:
                self.plan_kind = (self.plan_kind + 1) % 3
                self.run_t = None
                self.run_playing = False
        elif key in (pygame.K_s, pygame.K_RIGHT):
            self.paused = True
            self.do_step()
        elif key == pygame.K_f:
            self.run_to_end()
        elif key == pygame.K_r:
            self.restart()
        elif key == pygame.K_l:
            self.paused = True
            self.open_browser()
        elif key == pygame.K_m:
            e = self.current_entry()
            if e:
                mc.toggle_favorite(self.prefs, e)
        elif key in (pygame.K_n, pygame.K_b):
            step = 1 if key == pygame.K_n else -1
            if self.maze_file:
                self.maze_file = self.neighbor_file(step)
            else:
                self.seed = max(0, self.seed + step)
            self.load_maze()
        elif key == pygame.K_a:
            self.algo = sl.ALGO_ADACHI if self.algo == sl.ALGO_DIJKSTRA else sl.ALGO_DIJKSTRA
            self.restart()
        elif key == pygame.K_x:
            self.start_plant()
        elif key == pygame.K_z:
            self.open_plant_log()
        elif key == pygame.K_o:
            # 探索の行き先: 往復 → 片道 → 全面 → 往復 …(最初から)
            order = [sl.SCOPE_ROUND, sl.SCOPE_ONE_WAY, sl.SCOPE_FULL]
            self.scope = order[(order.index(self.scope) + 1) % len(order)]
            self.apply_search()
            self.restart()
        elif key == pygame.K_k:
            # 最短走行の速度帯: 次の帯(最後の次は1)。探索が終わっていれば経路と時間を計算し直す
            self.band = self.sim.set_band(self.band % self.sim.band_count() + 1)
            self.prof = self.sim.run_profile()
            self.run_t = None
            self.run_playing = False
            if self.plans is not None:
                self.compute_plans()
        elif key == pygame.K_c:
            self.show_cost = not self.show_cost
        elif key == pygame.K_t:
            self.show_hidden = not self.show_hidden
        elif key == pygame.K_UP:
            if self.run_t is not None:
                self.run_scale = min(4.0, self.run_scale * 2)
            else:
                self.speed = min(200, self.speed * 2)
        elif key == pygame.K_DOWN:
            if self.run_t is not None:
                self.run_scale = max(1 / 16, self.run_scale / 2)
            else:
                self.speed = max(1, self.speed // 2)
        return True

    def repeat_browser_key(self, dt):
        """迷路の一覧で移動キーを押し続けている間、一定の間隔で同じ移動を繰り返す"""
        if self.repeat_key is None:
            return
        # 一覧を閉じた・キーを離した(KEYUP を取りこぼした場合も)ら止める
        if not self.browser_open or not pygame.key.get_pressed()[self.repeat_key]:
            self.repeat_key = None
            return
        self.repeat_timer -= dt
        while self.repeat_timer <= 0:
            self.browser_key(self.repeat_key)
            self.repeat_timer += REPEAT_INTERVAL_S

    def run(self):
        running = True
        while running:
            dt = self.clock.tick(FPS) / 1000.0
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    in_browser = self.browser_open
                    running = self.handle_key(event.key)
                    if in_browser and event.key in REPEAT_KEYS:
                        self.repeat_key = event.key
                        self.repeat_timer = REPEAT_DELAY_S
                elif event.type == pygame.KEYUP and event.key == self.repeat_key:
                    self.repeat_key = None

            self.repeat_browser_key(dt)
            self.poll_plant()

            motion = self.current_motion()
            if self.run_playing and motion is not None and not self.browser_open:
                self.run_t = min(motion.duration, self.run_t + dt * self.run_scale)
                if self.run_t >= motion.duration:
                    self.run_playing = False

            interval = 1.0 / self.speed
            self.anim += dt / min(interval, 0.25)
            if not self.paused and not self.finished() and not self.browser_open:
                self.step_timer += dt
                while self.step_timer >= interval and not self.finished():
                    self.step_timer -= interval
                    self.do_step()
                if self.speed > 30:
                    self.anim = 1.0  # 速いときは補間しない

            self.draw()
        pygame.quit()


def main():
    p = argparse.ArgumentParser(description="GUI for the yuho maze logic (same sources as the firmware)")
    p.add_argument("maze_file", nargs="?", help="maze file (classic text format)")
    p.add_argument("--random", type=int, default=None, help="seed of the random maze")
    p.add_argument("--algo", choices=sl.ALGO_NAMES, default="dijkstra")
    p.add_argument("--goal", type=parse_cost, default=None, help="cost to goal: S,T90,T180,K")
    p.add_argument("--back", type=parse_cost, default=None, help="cost back to start: S,T90,T180,K")
    p.add_argument("--speed", type=int, default=8, help="cells per second (default 8)")
    p.add_argument("--scope", choices=["round", "oneway", "full"], default="round",
                   help="search scope: round trip, one way, full (default round)")
    p.add_argument("--search", type=parse_search, default=None,
                   help="search speeds V,TURN_V,ACCEL [mm/s, mm/s, mm/s^2] (default params.h)")
    p.add_argument("--pivot", action="store_true", help="search turns by pivoting instead of slalom")
    p.add_argument("--band", type=int, default=3, help="fastest-run speed band in FAST_BANDS (default 3)")
    p.add_argument("--plant-no-build", action="store_true", help="X: do not rebuild plant_sim before running it")
    p.add_argument("--plant-opt", default=None, help='X: options passed to plant_sim as is (e.g. "--no-crash-stop")')
    args = p.parse_args()
    MazeGui(args).run()


if __name__ == "__main__":
    main()
